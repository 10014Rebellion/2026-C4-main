import { TABLE, KEYS, CONST } from "./telemetryKeys.js";
import { NT4_Client } from "./nt4.js";

const batteryEl = document.getElementById("battery");
const stateEl = document.getElementById("state");
const shooterEl = document.getElementById("shooter");
const overlay = document.getElementById("lowBattOverlay");

let enabled = false;
let auto = false;

const topicBattery = `/${TABLE}/${KEYS.BATTERY}`;
const topicEnabled = `/${TABLE}/${KEYS.ENABLED}`;
const topicAuto = `/${TABLE}/${KEYS.AUTO}`;
const topicShooter = `/${TABLE}/${KEYS.SHOOTER_RPS}`;

function updateStateText() {
  stateEl.textContent = !enabled ? "DISABLED" : (auto ? "AUTO" : "TELEOP");
}

function handleBattery(v) {
  batteryEl.textContent = v.toFixed(2) + " V";
  const low = v < CONST.LOW_BATT_WARN_V;
  overlay.hidden = !low;
  document.body.classList.toggle("low-batt", low);
}

function handleValue(topic, value) {
  if (topic === topicBattery) return handleBattery(value);
  if (topic === topicEnabled) { enabled = !!value; return updateStateText(); }
  if (topic === topicAuto) { auto = !!value; return updateStateText(); }
  if (topic === topicShooter) { shooterEl.textContent = Math.round(value) + " rps"; }
}

const nt = new NT4_Client(window.location.hostname, "DriverDashboard");
nt.subscribe([topicBattery, topicEnabled, topicAuto, topicShooter]);

nt.onNewValue = (topic, value) => handleValue(topic, value);

nt.connect();
updateStateText();