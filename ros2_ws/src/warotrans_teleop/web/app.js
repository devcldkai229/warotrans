(() => {
  "use strict";

  const HEARTBEAT_MS = 66; // ~15 Hz
  const STATUS_POLL_MS = 125; // ~8 Hz

  let activeDirection = null;
  let heartbeatTimer = null;
  let currentSpeedPercent = 20;

  const els = {
    rosStatus: document.getElementById("ros-status"),
    commandStatus: document.getElementById("command-status"),
    speedStatus: document.getElementById("speed-status"),
    speedLevel: document.getElementById("speed-level"),
    linearCmd: document.getElementById("linear-cmd"),
    angularCmd: document.getElementById("angular-cmd"),
    odomLinear: document.getElementById("odom-linear"),
    odomAngular: document.getElementById("odom-angular"),
    leftPwm: document.getElementById("left-pwm"),
    rightPwm: document.getElementById("right-pwm"),
    lastCommand: document.getElementById("last-command"),
  };

  async function postJson(url, body) {
    const response = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    return response.json();
  }

  async function fetchStatus() {
    try {
      const response = await fetch("/api/status", { cache: "no-store" });
      if (!response.ok) {
        throw new Error(`HTTP ${response.status}`);
      }
      const status = await response.json();
      updateStatus(status);
      els.rosStatus.textContent = "CONNECTED";
    } catch (_err) {
      els.rosStatus.textContent = "DISCONNECTED";
    }
  }

  function updateStatus(status) {
    currentSpeedPercent = status.speed_level_percent;
    els.commandStatus.textContent = status.direction;
    els.speedStatus.textContent = `${status.speed_level_percent.toFixed(0)}%`;
    els.speedLevel.textContent = `${status.speed_level_percent.toFixed(0)}%`;
    els.linearCmd.textContent = `${status.linear_cmd.toFixed(3)} m/s`;
    els.angularCmd.textContent = `${status.angular_cmd.toFixed(3)} rad/s`;
    els.odomLinear.textContent = `${status.odom_linear.toFixed(3)} m/s`;
    els.odomAngular.textContent = `${status.odom_angular.toFixed(3)} rad/s`;
    els.leftPwm.textContent = `${status.left_pwm_percent.toFixed(0)}% (${status.left_pwm_raw}/${status.pwm_raw_max})`;
    els.rightPwm.textContent = `${status.right_pwm_percent.toFixed(0)}% (${status.right_pwm_raw}/${status.pwm_raw_max})`;
    els.lastCommand.textContent = `${status.last_command_age_ms} ms ago`;
  }

  function clearHeartbeat() {
    if (heartbeatTimer !== null) {
      clearInterval(heartbeatTimer);
      heartbeatTimer = null;
    }
  }

  async function sendCommand(direction) {
    const status = await postJson("/api/command", {
      direction,
      speed_percent: currentSpeedPercent,
    });
    updateStatus(status);
  }

  async function sendStop() {
    clearHeartbeat();
    activeDirection = null;
    document.querySelectorAll(".btn.drive.active").forEach((btn) => {
      btn.classList.remove("active");
    });
    const status = await postJson("/api/stop", {});
    updateStatus(status);
  }

  function startDrive(direction, button) {
    if (direction === "STOP") {
      sendStop();
      return;
    }

    activeDirection = direction;
    document.querySelectorAll(".btn.drive.active").forEach((btn) => {
      btn.classList.remove("active");
    });
    button.classList.add("active");

    clearHeartbeat();
    sendCommand(direction).catch(() => sendStop());
    heartbeatTimer = setInterval(() => {
      if (activeDirection) {
        sendCommand(activeDirection).catch(() => sendStop());
      }
    }, HEARTBEAT_MS);
  }

  function bindDriveButton(button) {
    const direction = button.dataset.direction;

    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      startDrive(direction, button);
    });

    const stopHandler = (event) => {
      event.preventDefault();
      if (button.hasPointerCapture(event.pointerId)) {
        button.releasePointerCapture(event.pointerId);
      }
      sendStop();
    };

    button.addEventListener("pointerup", stopHandler);
    button.addEventListener("pointercancel", stopHandler);
    button.addEventListener("lostpointercapture", stopHandler);
  }

  document.querySelectorAll(".btn.drive, .btn.stop").forEach((button) => {
    bindDriveButton(button);
  });

  document.getElementById("speed-down").addEventListener("click", async () => {
    try {
      const status = await postJson("/api/speed_delta", { delta: -5 });
      updateStatus(status);
      if (activeDirection) {
        await sendCommand(activeDirection);
      }
    } catch (_err) {
      /* ignore */
    }
  });

  document.getElementById("speed-up").addEventListener("click", async () => {
    try {
      const status = await postJson("/api/speed_delta", { delta: 5 });
      updateStatus(status);
      if (activeDirection) {
        await sendCommand(activeDirection);
      }
    } catch (_err) {
      /* ignore */
    }
  });

  document.querySelectorAll(".btn.preset").forEach((button) => {
    button.addEventListener("click", async () => {
      const speed = Number(button.dataset.speed);
      try {
        const status = await postJson("/api/speed_set", {
          speed_percent: speed,
        });
        updateStatus(status);
        if (activeDirection) {
          await sendCommand(activeDirection);
        }
      } catch (_err) {
        /* ignore */
      }
    });
  });

  ["contextmenu", "selectstart", "dragstart"].forEach((eventName) => {
    document.addEventListener(
      eventName,
      (event) => {
        if (event.target.closest(".btn")) {
          event.preventDefault();
        }
      },
      { passive: false }
    );
  });

  document.addEventListener("visibilitychange", () => {
    if (document.visibilityState !== "visible") {
      sendStop();
    }
  });

  window.addEventListener("pagehide", () => {
    sendStop();
  });

  setInterval(fetchStatus, STATUS_POLL_MS);
  fetchStatus();
})();
