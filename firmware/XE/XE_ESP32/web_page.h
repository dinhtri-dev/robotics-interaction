#pragma once
// Separate HTML avoids Arduino generating C++ prototypes from JavaScript.
static const char YEAR2_WEB_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP32-C3 Line Robot</title>

  <style>
    body {
      font-family: Arial;
      text-align: center;
      background: #f2f2f2;
      user-select: none;
    }

    .box {
      background: white;
      width: 90%;
      max-width: 420px;
      margin: 30px auto;
      padding: 25px;
      border-radius: 18px;
      box-shadow: 0 4px 12px rgba(0,0,0,0.15);
    }

    h2 {
      color: #222;
    }

    .btn {
      width: 260px;
      height: 90px;
      margin: 15px;
      font-size: 30px;
      border: none;
      border-radius: 16px;
      color: white;
      font-weight: bold;
    }

    .start {
      background: #2e7d32;
    }

    .start:active {
      background: #1b5e20;
      transform: scale(0.96);
    }

    .status {
      margin-top: 15px;
      font-size: 18px;
      color: #d32f2f;
      font-weight: bold;
    }

    .note {
      font-size: 18px;
      color: #333;
      margin-top: 15px;
      line-height: 1.5;
    }
  </style>
</head>

<body>
  <div class="box">
    <h2>ESP32-C3 → NANO LINE ROBOT</h2>

    <button class="btn start" onclick="sendCmd('1')">START LINE</button>

    <div class="note">
      Bấm START LINE để gửi lệnh số <b>1</b> sang Nano.
      <br><br>
      Nano sẽ bám line → gặp full đen → quay 180 → dừng.
    </div>

    <div id="status" class="status">READY</div>
  </div>

<script>
function sendCmd(cmd) {
  document.getElementById("status").innerHTML = "ĐANG GỬI LỆNH...";

  fetch('/cmd?key=' + cmd, {method: 'POST'})
    .then(response => response.text())
    .then(data => {
      document.getElementById("status").textContent = data;
      console.log(data);
    })
    .catch(err => {
      document.getElementById("status").innerHTML = "LỖI KẾT NỐI ESP32-C3";
      console.log(err);
    });
}
</script>

</body>
</html>
)rawliteral";
