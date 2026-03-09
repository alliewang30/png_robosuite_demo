from flask import Flask, request, jsonify, render_template_string

HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Kinova PNG Controller</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.10.1/nipplejs.min.js"></script>
  <style>
    body { font-family: sans-serif; margin: 20px; background: #f0f0f0; user-select: none; }
    
    .container { 
      display: flex; 
      gap: 40px; 
      align-items: center; 
      justify-content: center; 
      margin-top: 30px; 
      flex-wrap: wrap; 
    }

    .joystick-zone { 
      background: #ddd; 
      position: relative; 
      border: 2px solid #bbb; 
    }

    #move-zone { 
      width: 380px;  
      height: 380px; 
      border-radius: 50%;
    }

    #twist-zone, #gripper-zone { 
      width: 80px; 
      height: 180px; 
      border-radius: 12px; 
    }

    #move-zone .nipple .front {
        width: 43px !important;   
        height: 43px !important;  
        margin-left: -21.5px !important; 
        margin-top: -21.5px !important;  
        opacity: 0.8;
    }
    #move-zone .nipple .back {
        width: 320px !important;
        height: 320px !important;
        margin-left: -160px !important;
        margin-top: -160px !important;
    }

    #twist-zone .nipple .back, 
    #gripper-zone .nipple .back {
        width: 50px !important;
        height: 140px !important;
        border-radius: 10px !important;
        margin-left: -25px !important; 
        margin-top: -70px !important;
        border: none !important;
        opacity: 0.4;
    }

    #twist-zone .nipple .front,
    #gripper-zone .nipple .front {
        width: 40px !important;
        height: 40px !important;
        margin-left: -20px !important;
        margin-top: -20px !important;
    }

    .label { text-align: center; font-weight: bold; margin-bottom: 10px; font-size: 14px; }
    
    .status-box { text-align: center; margin: 20px auto; padding: 15px; background: #fff; width: 300px; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
    .mode-indicator { font-size: 20px; color: #007bff; font-weight: bold; }
    
    .controls { text-align: center; margin-top: 40px; }
        
    button { 
          padding: 12px 24px; 
          margin: 8px; 
          cursor: pointer; 
          font-weight: bold; 
          border: none;
          background: #ffffff;
          border-radius: 12px; 
          box-shadow: 0 4px 6px rgba(0,0,0,0.1); 
          transition: all 0.2s ease;
          color: #333;
          font-size: 16px;
        }
    
    button:hover { box-shadow: 0 2px 4px rgba(0,0,0,0.1); transform: translateY(1px); }
    .btn-quit { background: #ffcccc; color: #b30000; }
  </style>
</head>
<body>
  <h2 style="text-align:center;">PnG Virtual Controller</h2>

  <div class="status-box">
    <div>Control Mode:</div>
    <div id="mode-text" class="mode-indicator">Loading...</div>
  </div>

  <div class="container">
    <div>
      <div class="label">Move (X/Y)</div>
      <div id="move-zone" class="joystick-zone"></div>
    </div>

    <div>
      <div class="label">Twist (Z)</div>
      <div id="twist-zone" class="joystick-zone"></div>
    </div>

    <div>
      <div class="label">Gripper</div>
      <div id="gripper-zone" class="joystick-zone"></div>
    </div>
  </div>

  <div class="controls">
    <button onclick="postSimple('/mode')">Toggle Mode</button>
    <button onclick="postSimple('/home')">Home</button>
    <button onclick="postSimple('/quit')" class="btn-quit">Quit</button>
  </div>

<script>
let currentAxes = [0, 0, 0, 0, 0, 0];

function postAxes() {
    fetch('/axes', {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({axes: currentAxes})
    });
}

function postSimple(url) {
    fetch(url, {method: "POST"});
}

function setGripper(val) {
    fetch('/gripper', {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({value: val})
    });
}

setInterval(() => {
    fetch('/status')
        .then(response => response.json())
        .then(data => {
            const modeText = data.mode === 0 ? "Translation" : "Rotation";
            document.getElementById('mode-text').innerText = modeText;
        });
}, 200);

// NippleJS Initializations
const moveManager = nipplejs.create({
    zone: document.getElementById('move-zone'),
    mode: 'static', position: {left: '50%', top: '50%'}, 
    color: 'blue',
    size: 380 
});
moveManager.on('move', (evt, data) => {
    currentAxes[0] = data.vector.x;
    currentAxes[1] = data.vector.y;
    postAxes();
});
moveManager.on('end', () => {
    currentAxes[0] = 0; currentAxes[1] = 0; postAxes();
});

const twistManager = nipplejs.create({
    zone: document.getElementById('twist-zone'),
    mode: 'static', position: {left: '50%', top: '50%'}, 
    color: 'red', 
    lockY: true,
    size: 140
});
twistManager.on('move', (evt, data) => {
    currentAxes[5] = data.vector.y;
    postAxes();
});
twistManager.on('end', () => {
    currentAxes[5] = 0; postAxes();
});

const gripManager = nipplejs.create({
    zone: document.getElementById('gripper-zone'),
    mode: 'static', position: {left: '50%', top: '50%'}, 
    color: 'green', 
    lockY: true,
    size: 140
});
gripManager.on('move', (evt, data) => {
    // Applying the 0.5 sensitivity as in your original
    setGripper(data.vector.y * 0.5);
});
gripManager.on('end', () => {
    setGripper(0);
});
</script>
</body>
</html>
"""

def create_app(state):
    app = Flask(__name__)
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)

    @app.get("/")
    def index():
        return render_template_string(HTML)

    @app.get("/status")
    def status():
        return jsonify(mode=state.mode)

    @app.post("/axes")
    def axes():
        data = request.get_json(force=True)
        state.set_axes(data["axes"])
        return jsonify(ok=True)

    @app.post("/gripper")
    def gripper():
        data = request.get_json(force=True)
        state.set_gripper(data["value"])
        return jsonify(ok=True)

    @app.post("/mode")
    def mode():
        state.request_mode_toggle()
        return jsonify(ok=True)

    @app.post("/home")
    def home():
        state.request_home()
        return jsonify(ok=True)

    @app.post("/quit")
    def quit_():
        state.request_quit()
        return jsonify(ok=True)

    return app