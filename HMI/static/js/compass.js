/* compass.js – boat stays still, compass spins */
(function () {
  // let heading = "S", rudder = 'r', goal = 'f', speed = null; // initial values
  let heading = 0, rudder = 0, goal = 0, speed = 0; // initial values
  let rudder_string = '----'
  let heading_string = '----';
  let speed_string = '----';
  let rpm_string = '----';
  let heading_goal_string = '----';
  let lat_string = '----';
  let lon_string = '----';
  let rudder_max = 0
  let rudder_min = 0
  let rudder_max_string = '----';
  let rudder_min_string = '----';
  let desired_goal = goal; // for the draggable handle
  const canvas = document.getElementById('compass');
  const ctx    = canvas.getContext('2d');
  let R      = canvas.height / 2;          // radius
  const RING_R = R;                    // everything (line & handle) uses this
  const boat   = new Image();
  boat.src     = '/static/images/boat.png'; // <- adjust if needed
  const HANDLE_R = 30; // radius of the draggable handle
  const dial       = { dragging:false };

  window.addEventListener('DOMContentLoaded', () => {
    console.log('DOMContentLoaded: compass.js');
    const socket = new WebSocket('ws://' + location.hostname + ':5000/ws');
  
    socket.onopen = function () {
        console.log("WebSocket from compass.js connected.");
    };
    
    socket.onclose = function () {
        console.log("WebSocket from compass.js disconnected.");
    };

    socket.onerror = function (error) {
        console.error("WebSocket from compass.js error:", error);
    };
    
    
    socket.onmessage = function (event) {
      const data = JSON.parse(event.data);
            // 0x18F01D21: {"name": "steering", "last_time": time.time()},
            // 0x19F10D13: {"name": "rudder", "last_time": time.time()},
            // 0x09F8021C: {"name": "gps_position", "last_time": time.time()},
            // 0x18FEE81C: {"name": "vehicle_direction", "last_time": time.time()},
            // 0x0CF00400: {"name": "engine_control", "last_time": time.time()},
            // 0x09F8021C: {"name": "gps_speed", "last_time": time.time()},
            // 0x09F8011C: {"name": "gps_position_rapid", "last_time": time.time()},
      if (data.timeout){
        if (data.source_id === 0x18F01D21) {//steering
          updateSteeringUI('----', '----');
        }
        else if (data.source_id === 0x19F10D13) {//rudder
          rudder = 0;
          rudder_string = '----';
          rudder_min_string = '----';
          rudder_max_string = '----';
        }
        else if (data.source_id === 0x09F8011C) {
          lat_string = '----';
          lon_string = '----';
        }
        else if (data.source_id === 0x09F8021C) {
          speed_string = '----';
          heading_string = '----';
        }
        else if (data.source_id === 0x0CF00400) {
          rpm_string = '----';
          if (engineSpeed) { engineSpeed.textContent = rpm_string;}
        }
      } 
      
      if ('rpm' in data) {
        try {
          rpm_string = data.rpm.toFixed(0).padStart(4, ' ');;
        } catch (error) {
          rpm_string = 'ERROR';
        }
        if (engineSpeed) { engineSpeed.textContent = rpm_string;}
      }

      if ('steering_angle' in data && 'steering_goal' in data && 'servo_enabled' in data) {
        try {
          angle_string = data.steering_angle.toFixed(0) + '°';
          servoEnabled = data.servo_enabled;
          setServoState(servoEnabled);
        } catch (error) {
          angle_string = 'ERROR';
        }
        
        let steering_goal_string = '****'; // default value
        if (data.servo_enabled) {
          try {
            steering_goal_string = data.steering_goal.toFixed(0) + '°';
          } catch (error) {
            steering_goal_string = 'ERROR';
          }
        }
        else {
          steering_goal_string = '****';  
        }  
        
        updateSteeringUI(angle_string, steering_goal_string);

      }
      
      if ('rudder_angle' in data) {
        try {
          rudder = data.rudder_angle;
          rudder_string = data.rudder_angle.toFixed(0) + '°';
        } catch (error) { 
          rudder = 0;
          rudder_string = 'ERROR';
        }
      }
      
      if ('rudder_value_min' in data && 'rudder_value_max' in data) {
        try {
          rudder_min_string = data.rudder_value_min;
          rudder_max_string = data.rudder_value_max;
        } catch (error) {
          rudder_min_string = 'ERROR';
          rudder_max_string = 'ERROR';
        }
        
      }
  
      if ('COG' in data && 'SOG' in data) {
        try {
          heading = data.COG;
          heading_string = fmtDeg(heading, 0);
          speed = data.SOG;
          speed_string = fmtSpd(speed, 1);
        } catch (error) {
          heading_string = 'ERROR';
          speed_string = 'ERROR';
        }
      }

      if ('heading_goal' in data) {
        try {
          goal = data.heading_goal; // goal is in degrees 
          heading_goal_string = fmtDeg(goal, 0);
        } catch (error) {
          heading_goal_string = 'ERROR';
        }
      }
    
    }
    const engineSpeed = document.getElementById('EngineSpeed');
    const servoBtn = document.getElementById("servoToggleBtn");
    const servoStatus = document.getElementById("servoEnabled");
    const rudderButtons = [
        document.getElementById("btnPortRudder"),
        document.getElementById("btnStarRudder")
    ];

    function setServoState(enabled) {
        if (enabled) {
            servoBtn.textContent = "Disable Servo";
            servoBtn.classList.remove("disabled");
            servoBtn.classList.add("enabled");
            servoStatus.textContent = "Servo is Enabled";
            rudderButtons.forEach(btn => btn.disabled = false);
        } else {
            servoBtn.textContent = "Enable Servo";
            servoBtn.classList.remove("enabled");
            servoBtn.classList.add("disabled");
            servoStatus.textContent = "Servo is Disabled";
            rudderButtons.forEach(btn => btn.disabled = true);
        }
    }

    function updateSteeringUI(angle_string, steering_goal_string) {
      const steeringValueEl = document.getElementById('steeringValue');
      const steeringGoalEl = document.getElementById('steeringGoal');
      if (steeringValueEl) { steeringValueEl.textContent = angle_string;}
      if (steeringGoalEl)  { steeringGoalEl.textContent  = steering_goal_string; }
    }

    document.getElementById('btnPortRudder').addEventListener('click', function () {
        socket.send(JSON.stringify({ command: "rudder_left" }));
        console.log("Port Rudder button clicked");
    });

    document.getElementById('btnStarRudder').addEventListener('click', function () {
        socket.send(JSON.stringify({ command: "rudder_right" }));
        console.log("Starboard Rudder button clicked");
    });

    let servoEnabled = false;

    servoBtn.addEventListener("click", () => {
        servoEnabled = !servoEnabled;
        setServoState(servoEnabled);
        if (servoEnabled) {
            // Send the new state to the server
            socket.send(JSON.stringify({ command: "servo_enable"}));
        }
        else {
            // Send the new state to the server
            socket.send(JSON.stringify({ command: "servo_disable"}));
        } 
    });

    servoToggleBtn.classList.add("disabled");

    /* set up buttons for goal adjustment*/
    const btnPort  = document.getElementById('btnPortGoal');
    const btnStar  = document.getElementById('btnStarGoal');

    if (btnPort) {
      btnPort.addEventListener('click', () => {
        socket.send(JSON.stringify({ command: "heading_left" }));
      });
    }

    if (btnStar) {
      btnStar.addEventListener('click', () => {
        socket.send(JSON.stringify({ command: "heading_right" }));
      });
    }

    function moveDrag(evt) {
      if (!dial.dragging) return;
      const { x, y } = pointerToCanvas(evt);

      /* Screen angle (0° = up) */
      const dx = x;
      const dy = -y;
      let ang  = Math.atan2(dx, dy) * 180 / Math.PI;    
      desired_goal = clamp360(ang+heading); 
      socket.send(JSON.stringify({ heading_goal: desired_goal }));
      
    }

    window.addEventListener('mousemove',  e => moveDrag(e));
    window.addEventListener('touchmove',  e => moveDrag(e.touches[0]));
    canvas.addEventListener('mousedown',  e => tryStartDrag(e));
    canvas.addEventListener('touchstart', e => tryStartDrag(e.touches[0]));
    window.addEventListener('mouseup',    () => dial.dragging = false);
    window.addEventListener('touchend',   () => dial.dragging = false);
    canvas.addEventListener('dblclick',  e => handleDblClick(e));       // mouse
    canvas.addEventListener('touchend',  e => {
      if (e.detail === 2) handleDblClick(e.changedTouches[0]);
    }); // touch

    function tryStartDrag(evt) {
      const { x, y } = pointerToCanvas(evt);
      const hit = hitHandle(x, y);
      // console.log('[tryStartDrag]', { x, y, R, hit });
      
      if (hit) {
        dial.dragging = true;
        moveDrag(evt);
      }
    }

    function handleDblClick(evt) {
      const { x, y } = pointerToCanvas(evt);

      /* Only react if the pointer is inside the handle */
      if (!hitHandle(x, y)) {
        desired_goal = heading;
        socket.send(JSON.stringify({ heading_goal: desired_goal }));
      }
    }

    

    if (socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ heading_goal: desired_goal }));
    } else {
        socket.addEventListener('open', function () {
            socket.send(JSON.stringify({ heading_goal: desired_goal }));
        }, { once: true });
    }


    console.log('compass.js: WebSocket and event listeners set up.');
});
  
  const dpr = window.devicePixelRatio || 1;

  function resizeCanvas () {
    const parent = canvas.parentElement;                // .center-col
    const { width:pw, height:ph } = parent.getBoundingClientRect();
    const sideCSS = Math.min(pw, ph);                   // largest square

    /* A.  Size the element in CSS pixels */
    canvas.style.width  = sideCSS + 'px';
    canvas.style.height = sideCSS + 'px';

    /* B.  Resize the drawing buffer in *device* pixels */
    canvas.width  = sideCSS * dpr;
    canvas.height = sideCSS * dpr;

    /* C.  Scale the context so your existing draw code still
          works in plain CSS coordinates */
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    /* D.  Update radius for all drawing calculations */
    R = sideCSS / 2;
  }





  /* -------------------------------------------------------------- *
  * Pointer → canvas-pixel coords with origin at canvas centre     *
  * ( axis orientation unchanged: +x right, +y down )              *
  * -------------------------------------------------------------- */
  function pointerToCanvas(evt) {
    const rect   = canvas.getBoundingClientRect();   // CSS pixel box
    const scaleX = canvas.width  / rect.width;       // DPR × zoom
    const scaleY = canvas.height / rect.height;

    // raw top-left–origin coords
    const cx = (evt.clientX - rect.left) * scaleX;
    const cy = (evt.clientY - rect.top ) * scaleY;

    // shift origin to centre
    return {
      x : cx - canvas.width  / 2,   // centre has x = 0
      y : cy - canvas.height / 2    // centre has y = 0
    };
  }


  /* put this once near the top of the file */
  function fmtDeg(val, digits = 3) {
    try {
      if (typeof val !== 'number' || isNaN(val)) throw new Error('bad');
      return val.toFixed(0).padStart(digits, '0') + '°';
    } catch (e) {
      return '---';
    }
  }

  function fmtSpd(val, digits = 3) {
    try {
      if (typeof val !== 'number' || isNaN(val)) throw new Error('bad');
      return val.toFixed(1).padStart(digits, ' ');
    } catch (e) {
      return '---';
    }
  }


  /* Pointer inside the circular handle? */
  function hitHandle(px, py) {
    const dx = px - 0;
    const dy = py;
    const r = Math.sqrt(dx*dx + dy*dy);
    const hitR = r > (0.8 * R * dpr)
    const hitY = py < 0
    console.log('[hitHandle]', { px, py, dx, dy, r, R, hitR, hitY });
    /* if distance from centre is greater than half radius, it’s a hit */
    return hitR && hitY; // only hit if above the centre
  }

  /* Keep any angle in 0-359 range */
  function clamp360(a) {                     // convenience normaliser
    return (a % 360 + 360) % 360;
  }

  /* Run once after the page loads, then on every resize/rotate */
    /* run after DOM is parsed & on every resize */
  window.addEventListener('DOMContentLoaded', resizeCanvas);
  window.addEventListener('resize',           resizeCanvas);
  window.addEventListener('load',   resizeCanvas);
  


  function draw() {
    // 1. Reset transform so clearRect wipes the whole buffer
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    heading = clamp360(heading);  // ensure heading is in [0, 360)
    goal    = clamp360(goal);     // ensure goal is in [0, 360) 
    // ── draw rotating compass ring ───────────────
    ctx.save();
    ctx.translate(R, R);
    ctx.rotate(-heading * Math.PI / 180);   // spin ring opposite to boat
    drawRing(ctx, R);
    ctx.restore();

    // ── draw static boat + moving rudder ─────────
    ctx.save();
    ctx.translate(R, R);
    
    // 1) boat sprite (scaled to ~70 % of canvas diameter)
    const scale = 0.4;
    const bw    = canvas.width * scale;
    const bh    = boat.height * (bw / boat.width);
    /* >>> 2) AXIS LINE (new) <<< */
    ctx.strokeStyle = 'rgba(87, 87, 87, 0.8)';  // 50 % alpha red
    ctx.lineWidth   = 3;
    ctx.beginPath();
    ctx.moveTo(0, R);   // 10 px beyond the bow
    ctx.lineTo(0,  -R);   // extend past the stern (adjust as you like)
    ctx.stroke();
    /* <<< end new block >>> */

    

    
    /* 3) SEMI-TRANSPARENT, TAPERED RUDDER  --------------------------- */
    ctx.save();
    ctx.translate(0, bh / 2 - 10);           // hinge at stern
    ctx.rotate(-rudder * Math.PI / 180);     // swing port / starboard

    const rudderLen  = bh * 0.30;            // tweak length (30 % of boat)
    const rudderBase = 24;                   // width at the hinge

    ctx.fillStyle = 'rgb(230, 85, 85)';  // 50 % alpha red
    ctx.beginPath();
    ctx.moveTo(-rudderBase / 2, 0);          // left hinge corner
    ctx.lineTo(rudderBase / 2, 0);           // right hinge corner
    ctx.lineTo(0, rudderLen);                // tip (forms the point)
    ctx.closePath();
    ctx.fill();

    ctx.restore();
    
    ctx.drawImage(boat, -bw / 2, -bh / 2, bw, bh);

    /* Draw Labels  */
    ctx.fillStyle   = 'rgb(12, 0, 249)'
    
    ctx.font        = '96px monospace'
    ctx.textAlign   = 'left';              // anchor on the right end
    ctx.textBaseline = 'top';               // anchor at the top edge
    ctx.fillText(heading_string,  // e.g. “347.6°”
                bw / 4 ,               // 8 px left of the boat’s left edge
                -bh / 4);                  // flush with the boat’s top
    
    ctx.font        = '50px monospace'
    ctx.textBaseline = 'bottom';               // anchor at the top edge
    ctx.fillText("Heading",  // e.g. “347.6°”
                bw / 4 ,               // 8 px left of the boat’s left edge
                -bh / 4);                  // flush with the boat’s top

    ctx.font        = '96px monospace'
    ctx.textAlign   = 'left';              // anchor on the right end
    ctx.textBaseline = 'top';               // anchor at the top edge
    ctx.fillText(speed_string,  // e.g. “347.6°”
                bw / 4 ,               // 8 px left of the boat’s left edge
                bh / 4);                  // flush with the boat’s top
    
    ctx.font        = '50px monospace'
    ctx.textBaseline = 'bottom';               // anchor at the top edge
    ctx.fillText("Speed",  // e.g. “347.6°”
                bw / 4 ,               // 8 px left of the boat’s left edge
                bh / 4);                  // flush with the boat’s top
    
    ctx.font        = '96px monospace'
    ctx.textAlign   = 'right';              // anchor on the right end
    ctx.textBaseline = 'top';               // anchor at the top edge
    ctx.fillText(rudder_string,  // e.g. “347.6°”
                -bw / 4 ,               // 8 px left of the boat’s left edge
                bh / 4);                  // flush with the boat’s top

    ctx.font        = '50px monospace'
    ctx.textBaseline = 'bottom';               // anchor at the top edge
    ctx.fillText("Rudder",  // e.g. “347.6°”
                -bw / 4 ,               // 8 px left of the boat’s left edge
                bh / 4);                  // flush with the boat’s top

    ctx.font        = '96px monospace'
    ctx.textAlign   = 'right';              // anchor on the right end
    ctx.textBaseline = 'top';               // anchor at the top edge
    ctx.fillText(heading_goal_string,  // e.g. “347.6°”
                -bw / 4 ,               // 8 px left of the boat’s left edge
                -bh / 4);                  // flush with the boat’s top
                
    ctx.font        = '50px monospace'
    ctx.textBaseline = 'bottom';               // anchor at the top edge
    ctx.fillText("Goal   ",  // e.g. “347.6°”
                -bw / 4 ,               // 8 px left of the boat’s left edge
                -bh / 4);                  // flush with the boat’s top

    ctx.font        = '50px monospace'
    ctx.textBaseline = 'bottom';               // anchor at the top edge
    ctx.fillText(rudder_min_string,  // e.g. “347.6°”
                -.4*bw,               // 8 px left of the boat’s left edge
                0.5*bh);                  // flush with the boat’s top

    ctx.textAlign   = 'left';
    ctx.fillText(rudder_max_string,  // e.g. “347.6°”
                .4*bw,               // 8 px left of the boat’s left edge
                0.5*bh);                  // flush with the boat’s top
    /* ───────────────────────── */

    ctx.restore();
    
    ctx.save();
    ctx.translate(R, R); 
    
    ctx.beginPath(); 
    ctx.strokeStyle = '#444';
    ctx.lineWidth   = 1;
    ctx.moveTo(-R, 0);
    ctx.lineTo(R, 0);
    ctx.moveTo(0, -R)
    ctx.lineTo(0, R);
    ctx.stroke();
    ctx.restore();

    requestAnimationFrame(draw);
  }

  /* helper – draw ticks & cardinal letters */
  function drawRing(ctx, R) {
    ctx.strokeStyle = 'rgba(9, 237, 89, 1)';  // 50 % alpha green
    ctx.lineWidth   = 10;
    ctx.beginPath();
    ctx.moveTo(0,0);   // 10 px beyond the bow
    ctx.lineTo(R*Math.sin(goal* Math.PI / 180),  -R*Math.cos(goal* Math.PI / 180));   // extend past the stern (adjust as you like)
    ctx.stroke();

    /* GREEN GOAL LINE (already there) */
    ctx.strokeStyle = 'rgba(9, 237, 89, 1)';
    ctx.lineWidth   = 10;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.lineTo(R * Math.sin(goal * Math.PI/180),
              -R * Math.cos(goal * Math.PI/180));
    ctx.stroke();

    /* ★ NEW: round draggable handle ★ */
    ctx.fillStyle = 'rgba(9, 237, 89, 0.8)';    // slightly transparent green
    ctx.beginPath();
    ctx.arc(R * Math.sin(goal * Math.PI/180),
           -R * Math.cos(goal * Math.PI/180),
           HANDLE_R, 0, 2 * Math.PI);
    ctx.fill();

    // outer circle
    ctx.strokeStyle = '#444';
    ctx.lineWidth   = 6;
    ctx.beginPath();
    ctx.arc(0, 0, R - 10, 0, 2 * Math.PI);
    ctx.stroke();

    // ticks
    ctx.lineWidth = 2;
    for (let deg = 0; deg < 360; deg += 5) {
      const len = (deg % 45 === 0) ? 35 : (deg % 15 === 0) ? 20 : 10;
      const a   = deg * Math.PI / 180;
      ctx.beginPath();
      ctx.moveTo((R - 10) * Math.sin(a), -(R - 10) * Math.cos(a));
      ctx.lineTo((R - 10 - len) * Math.sin(a), -(R - 10 - len) * Math.cos(a));
      ctx.stroke();
    }

    // cardinal letters — always upright
    ctx.fillStyle   = '#222';
    ctx.font        = '48px sans-serif';
    ctx.textAlign   = 'center';
    ctx.textBaseline= 'middle';

    ['N','E','S','W'].forEach((ltr, i) => {
      const a  = i * Math.PI / 2;                    // 0, 90, 180, 270°
      const x  = (R - 70) * Math.sin(a);
      const y  = -(R - 70) * Math.cos(a);

      ctx.save();                                    // isolate transform
      ctx.translate(x, y);                           // move to label position
      ctx.rotate(heading * Math.PI / 180);           // cancel ring rotation
      ctx.fillText(ltr, 0, 0);                       // draw upright
      ctx.restore();
    });

  }
  // kick-off after boat sprite loads
  boat.onload = draw;
  
})();
