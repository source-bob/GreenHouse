'use strict';

const co2Ctx = document.getElementById('co2Chart');
const co2Chart = new Chart(co2Ctx, {
    type: 'line',
    data: {
        datasets: [
            {
                label: 'CO2',
                data: [],
                borderColor: 'rgb(47,143,186)',
                backgroundColor: 'rgb(47,143,186)',
                borderWidth: 2,
                lineTension: 0.3
            },
            {
                label: 'Set Point',
                data: [],
                borderColor: 'rgb(234,49,49)',
                backgroundColor: 'rgb(234,49,49)',
                borderWidth: 1
            }
        ]
    },
    options: {
        aspectRatio: 3,
        maintainAspectRatio: true,
        scales: {
            y: {
                beginAtZero: false
            }
        }
    }
});

const envCtx = document.getElementById('envChart');
const envChart = new Chart(envCtx, {
    type: 'line',
    data: {
        datasets: [
            {
                label: 'ºC',
                data: [],
                borderColor: 'rgb(204, 0, 102)',
                backgroundColor: 'rgb(204, 0, 102)',
                borderWidth: 2,
                lineTension: 0.3
            },
            {
                label: 'RH',
                data: [],
                borderColor: 'rgb(0, 153, 0)',
                backgroundColor: 'rgb(0, 153, 0)',
                borderWidth: 2,
                lineTension: 0.3
            },
            {
                label: 'Fan',
                data: [],
                borderColor: 'rgb(75, 192, 192)',
                backgroundColor: 'rgb(75, 192, 192)',
                borderWidth: 2
            }
        ]
    },
    options: {
        aspectRatio: 3,
        maintainAspectRatio: true,
        scales: {
            y: {
                beginAtZero: true
            }
        }
    }
});

const socket = new WebSocket(`ws://${location.host}`);
socket.addEventListener('open', () => {
    socket.send('WS Client Connected!');
    socket.addEventListener('message', event => {
        const msg = JSON.parse(event.data);
        const CO2 = msg.co2;
        const RH = msg.rh;
        const TMP = msg.tmp;
        const FAN = msg.fan;
        const SP = msg.sp;
        const tTime = msg.tTime;
        const co2 = document.querySelector('.co2');
        const rh = document.querySelector('.rh');
        const tmp = document.querySelector('.tmp');
        const time = document.querySelector('.time');
        const fan = document.querySelector('.fan');
        const sp = document.querySelector('.sp');
        rh.innerText = 'Relative humidity: ' + RH + '%';
        tmp.innerText = 'Temperature: ' + TMP + 'ºC';
        time.innerText = 'Time: ' + tTime;
        co2.innerText = 'CO2 Value: ' + CO2 + 'ppm';
        sp.innerText = 'Set Point: ' + SP + 'ppm';
        const currentSetpoint = document.getElementById("currentSetpoint");
        if (currentSetpoint) currentSetpoint.textContent = SP;
        fan.innerText = 'Fan Speed: ' + FAN + '%';
        addData(co2Chart, envChart, tTime, TMP, RH, CO2, FAN, SP);
    });
});

(() => {
    fetch('/historydata')
        .then(res => res.json())
        .then(gData => {
            for (const data of gData) {
                addData(co2Chart, envChart, data.tTime, data.tmp, data.rh, data.co2, data.fan, data.sp);
            }
        });
}).call({});

function addData(co2Chart, envChart, time, tmp, rh, co2, fan, sp) {
    co2Chart.data.labels.push(time);
    co2Chart.data.datasets[0].data.push(co2);
    co2Chart.data.datasets[1].data.push(sp);
    co2Chart.update();

    envChart.data.labels.push(time);
    envChart.data.datasets[0].data.push(tmp);
    envChart.data.datasets[1].data.push(rh);
    envChart.data.datasets[2].data.push(fan);
    envChart.update();
}

async function refreshRemoteSetpoint() {
  try {
    const res = await fetch("/api/setpoint");
    if (!res.ok) throw new Error("GET /api/setpoint failed");

    const rawText = await res.text();

    let data = null;
    try {
      data = JSON.parse(rawText);
    } catch (_) {
      data = { sp: parseInt(rawText, 10) };
    }

    if (!data || !Number.isInteger(data.sp)) {
      throw new Error("Invalid setpoint response");
    }

    const currentEl = document.getElementById("currentSetpoint");
    const inputEl = document.getElementById("setpointInput");

    if (currentEl) currentEl.textContent = data.sp;
    if (inputEl && document.activeElement !== inputEl) {
      inputEl.value = data.sp;
    }
  } catch (err) {
    console.error(err);
  }
}

async function submitRemoteSetpoint() {
  const inputEl = document.getElementById("setpointInput");
  const statusEl = document.getElementById("setpointStatus");
  const currentEl = document.getElementById("currentSetpoint");

  if (!inputEl || !statusEl) return;

  const sp = parseInt(inputEl.value, 10);

  if (!Number.isInteger(sp) || sp < 200 || sp > 1500) {
    statusEl.textContent = "Invalid target. Use 200...1500 ppm.";
    return;
  }

  try {
    statusEl.textContent = "Sending...";

    const body = new URLSearchParams({ sp: String(sp) });

    const res = await fetch("/api/setpoint", {
      method: "POST",
      headers: {
        "Content-Type": "application/x-www-form-urlencoded",
      },
      body: body.toString(),
    });

    const rawText = await res.text();

    let data = null;
    try {
      data = JSON.parse(rawText);
    } catch (_) {
      data = { ok: res.ok, sp: parseInt(rawText, 10) };
    }

    if (!res.ok || !data || !Number.isInteger(data.sp)) {
      throw new Error((data && data.error) || "POST /api/setpoint failed");
    }

    statusEl.textContent = `Command queued: ${data.sp} ppm`;
    setTimeout(() => {
    refreshRemoteSetpoint();
    }, 4000);
  } catch (err) {
    console.error(err);
    statusEl.textContent = "Failed to update target";
  }
}

document.addEventListener("DOMContentLoaded", () => {
  const btn = document.getElementById("setpointBtn");
  const input = document.getElementById("setpointInput");

  if (btn) {
    btn.addEventListener("click", submitRemoteSetpoint);
  }

  if (input) {
    input.addEventListener("keydown", (e) => {
      if (e.key === "Enter") {
        submitRemoteSetpoint();
      }
    });
  }

  refreshRemoteSetpoint();
});