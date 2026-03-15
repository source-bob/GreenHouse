"use strict";

const path = require("path");
const { WebSocketServer } = require("ws");
const http = require("http");
const express = require("express");
const app = express();
app.use(express.urlencoded({ extended: false }));
app.use(express.json());

const THINGSPEAK_CHANNEL_ID = "MY_THINGSPEAK_CHANNEL_ID";
const THINGSPEAK_READ_API_KEY = process.env.THINGSPEAK_READ_API_KEY || "MY_THINGSPEAK_API_KEY";

const THINGSPEAK_TALKBACK_ID = "MY_THINGSPEAK_TACKBACK_ID";
const THINGSPEAK_TALKBACK_API_KEY = process.env.THINGSPEAK_TALKBACK_API_KEY || "MY_THINGSPEAK_TALKBACK_API_KEY";

const THINGSPEAK_POLL_MS = 5000;
const THINGSPEAK_HISTORY_RESULTS = 120;

let port = 3003;

let server = http.createServer(app).listen(port);
const moment = require("moment");

app.use("/style", express.static(path.resolve(__dirname, "style")));
app.use("/static", express.static(path.resolve(__dirname, "static")));

const webSocketServer = new WebSocketServer({
  server: server,
  clientTracking: true,
});

webSocketServer.on("connection", (webSocket) => {
  console.log("WebSocket Connected");
  webSocket.on("message", (message) => {
    console.log(message.toString());
  });
});

let latestCache = {
  tTime: "--:--:--",
  co2: 0,
  rh: 0,
  tmp: 0,
  fan: 0,
  sp: 800,
  entry_id: 0,
};

function mapThingSpeakFeed(feed) {
  return {
    tTime: feed?.created_at ? moment(feed.created_at).local().format("HH:mm:ss") : "--:--:--",
    co2: parseInt(feed?.field1 ?? "0", 10) || 0,
    rh:  parseInt(feed?.field2 ?? "0", 10) || 0,
    tmp: parseInt(feed?.field3 ?? "0", 10) || 0,
    fan: parseInt(feed?.field4 ?? "0", 10) || 0,
    sp:  parseInt(feed?.field5 ?? "0", 10) || 0,
    entry_id: Number(feed?.entry_id ?? 0) || 0,
  };
}

function publishGreenhouseData(greenhouseData) {
  latestCache = greenhouseData;

  for (const ws of webSocketServer.clients) {
    try {
      ws.send(JSON.stringify(greenhouseData));
    } catch (e) {
      console.log("WS send failed:", e.message);
    }
  }
}

async function pollThingSpeakAndBroadcast() {
  try {
    const latest = await fetchThingSpeakLatest();

    if (latest.entry_id !== latestCache.entry_id) {
      publishGreenhouseData(latest);
    } else {
      latestCache = latest;
    }
  } catch (e) {
    console.log("ThingSpeak poll failed:", e.message);
  }
}

setInterval(pollThingSpeakAndBroadcast, THINGSPEAK_POLL_MS);
pollThingSpeakAndBroadcast();

async function fetchThingSpeakLatest() {
  const url =
    `https://api.thingspeak.com/channels/${THINGSPEAK_CHANNEL_ID}/feeds/last.json` +
    `?api_key=${encodeURIComponent(THINGSPEAK_READ_API_KEY)}`;

  const res = await fetch(url);
  if (!res.ok) throw new Error(`ThingSpeak latest failed: ${res.status}`);

  const feed = await res.json();
  return mapThingSpeakFeed(feed);
}

async function fetchThingSpeakHistory(results = THINGSPEAK_HISTORY_RESULTS) {
  const safeResults = Math.min(Math.max(results, 1), 8000);

  const url =
    `https://api.thingspeak.com/channels/${THINGSPEAK_CHANNEL_ID}/feeds.json` +
    `?api_key=${encodeURIComponent(THINGSPEAK_READ_API_KEY)}` +
    `&results=${safeResults}`;

  const res = await fetch(url);
  if (!res.ok) throw new Error(`ThingSpeak history failed: ${res.status}`);

  const data = await res.json();
  return (data.feeds || []).map(mapThingSpeakFeed);
}

app.get("/api/setpoint", async (req, res) => {
  try {
    latestCache = await fetchThingSpeakLatest();
    res.type("text/plain").send(String(latestCache.sp));
  } catch (e) {
    console.log(e);
    res.status(500).json({ ok: false, error: "READ SP FAILED" });
  }
});

app.get("/", async (req, res) => {
  try {
    latestCache = await fetchThingSpeakLatest();
  } catch (e) {
    console.log("Initial latest fetch failed:", e.message);
  }

  res.render(path.resolve(__dirname, "views/home.ejs"), latestCache);
});

app.get("/historydata", async (req, res) => {
  try {
    const history = await fetchThingSpeakHistory(THINGSPEAK_HISTORY_RESULTS);
    res.json(history);
  } catch (e) {
    console.log(e);
    res.status(500).send("ThingSpeak history error");
  }
});

app.post("/api/setpoint", async (req, res) => {
  const raw = req.body.sp ?? req.query.sp;
  const sp = parseInt(raw, 10);

  if (!Number.isInteger(sp) || sp < 200 || sp > 1500) {
    return res.status(400).json({ ok: false, error: "BAD SP" });
  }

  try {
    const body = new URLSearchParams({
      api_key: THINGSPEAK_TALKBACK_API_KEY,
      command_string: `SP=${sp}`,
    });

    const tsRes = await fetch(
      `https://api.thingspeak.com/talkbacks/${THINGSPEAK_TALKBACK_ID}/commands.json`,
      {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded" },
        body: body.toString(),
      }
    );

    const text = await tsRes.text();
    if (!tsRes.ok) {
      throw new Error(`TalkBack add failed: ${tsRes.status} ${text}`);
    }

    res.json({ ok: true, sp });
  } catch (e) {
    console.log(e);
    res.status(500).json({ ok: false, error: "TALKBACK POST FAILED" });
  }
});

app.get("/home.js", (req, res) =>
  res.sendFile(path.resolve(__dirname, "views/home.js"))
);
app.get("/logo.png", (req, res) =>
  res.sendFile(path.resolve(__dirname, "static/logo.png"))
);
app.get("/style.css", (req, res) =>
  res.sendFile(path.resolve(__dirname, "style/style.css"))
);
//app.listen(3000);
