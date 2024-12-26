// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const matchTimer = "/SmartDashboard/Match Time"
const FMSInfo = "/FMSInfo/.type"

function updateMatchTimer(time) {
  var matchTime = parseInt(time, 10).toString()
  if (matchTime < 0) {
    matchTime = 3005
  }
  document.getElementsByName("timer")[0].innerHTML = matchTime
}

function updateStatus(status) {
  document.getElementsByName("connection_status")[0].innerHTML = status
}


let client = new NT4_Client(
  window.location.hostname,
  "MatchTimer",
  (topic) => {
    // Topic announce
  },
  (topic) => {
    // Topic unannounce
  },
  (topic, timestamp, value) => {
    // New data
    if (topic.name === FMSInfo) {
      //document.body.style.backgroundColor = "grey";
    } else if (topic.name == matchTimer) {
      updateMatchTimer(value);
    }
  },
  () => {
    // Connected
    updateStatus("Connected")
  },
  () => {
    // Disconnected
    updateStatus("Disconnected")
    updateMatchTimer(-1)
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe(
    [matchTimer, FMSInfo],
    false,
    false,
    0.25
  );
  client.connect();
});
