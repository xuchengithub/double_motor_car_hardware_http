#include <SimpleFOC.h>
#include <WiFi.h>
#include <WebServer.h>

// WiFi credentials
const char* ssid = "OPPOchen";
const char* password = "xuchenxuchen";

// Create a web server on port 80
WebServer server(80);

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(17, 5, 18, 19);
BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(27, 26, 14, 25);

// Command settings
float target_velocity = -2; // Initial target velocity for both motors

// void setup() {
//   Serial.begin(115200);

//   // Initialize WiFi
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   Serial.println("WiFi connected");
  
//   // Print the IP Address
//   Serial.print("IP Address: ");
//   Serial.println(WiFi.localIP());

//   // Setup HTTP routes for Motor 0
//   server.on("/velocity0", HTTP_GET, []() {
//     if (server.hasArg("value")) {
//       motor.target = server.arg("value").toFloat();
//       server.send(200, "text/plain", "Motor 0 velocity updated");
//     } else {
//       server.send(400, "text/plain", "Bad request for Motor 0");
//     }
//   });
//   server.on("/speed0", HTTP_GET, []() {
//     server.send(200, "text/plain", String(motor.shaft_velocity));
//   });

//   // Setup HTTP routes for Motor 1
//   server.on("/velocity1", HTTP_GET, []() {
//     if (server.hasArg("value")) {
//       motor1.target = server.arg("valu").toFloat();
//       server.send(200, "text/plain", "Motor 1 velocity updated");
//     } else {
//       server.send(400, "text/plain", "Bad request for Motor 1");
//     }
//   });
//   server.on("/speed1", HTTP_GET, []() {
//     server.send(200, "text/plain", String(motor1.shaft_velocity));
//   });

//   server.begin();

//   // Motor setup...
//   I2Cone.begin(4, 16, 400000); 
//   I2Ctwo.begin(32, 33, 400000);
//   sensor.init(&I2Cone);
//   sensor1.init(&I2Ctwo);
//   motor.linkSensor(&sensor);
//   motor1.linkSensor(&sensor1);
//   driver.voltage_power_supply = 7.6;
//   driver.init();
//   driver1.voltage_power_supply = 7.6;
//   driver1.init();
//   motor.linkDriver(&driver);
//   motor1.linkDriver(&driver1);
//   motor.foc_modulation = motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
//   motor.controller = motor1.controller = MotionControlType::velocity;
//   motor.PID_velocity.P = motor1.PID_velocity.P = 0.2;
//   motor.PID_velocity.I = motor1.PID_velocity.I = 10;
//   motor.voltage_limit = motor1.voltage_limit = 7.6;
//   motor.LPF_velocity.Tf = motor1.LPF_velocity.Tf = 0.01;
//   motor.velocity_limit = motor1.velocity_limit = 40;
//   motor.init();
//   motor1.init();
//   motor.initFOC();
//   motor1.initFOC();
// }

// void loop() {
//   motor.loopFOC();
//   motor.move(motor.target); // Use separate target variable for each motor
//   motor1.loopFOC();
//   motor1.move(motor1.target); // Use separate target variable for each motor

//   server.handleClient(); // Handle client requests
// }
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void setup() {
  Serial.begin(115200);

  // 初始化 WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // 打印 IP 地址
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // 初始化硬件和服务器
  initHardware();
  initServer();

  // 创建两个任务，分别绑定到ESP32的两个核心
  xTaskCreatePinnedToCore(serverTask, "ServerTask", 10000, NULL, 1, NULL, 0); // 绑定到核心0
  xTaskCreatePinnedToCore(motorControlTask, "MotorControlTask", 10000, NULL, 1, NULL, 1); // 绑定到核心1
}

void loop() {
  // 在setup中创建了所有必要的任务后，loop可以为空
}

// 运行在核心0的任务
void serverTask(void * pvParameters) {
  server.begin();
  while(1) {
    server.handleClient();
    delay(1); // 让出控制权以避免WDT复位
  }
}

// 运行在核心1的任务
void motorControlTask(void * pvParameters) {
  while(1) {
    motor.loopFOC();
    motor.move(motor.target);
    motor1.loopFOC();
    motor1.move(motor1.target);
    // delay(1); // 可调整延时以匹配电机控制的需求
  }
}

void initHardware() {
  // 初始化电机和传感器等硬件接口
  I2Cone.begin(4, 16, 400000);
  I2Ctwo.begin(32, 33, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);
  driver.voltage_power_supply = 7.6;
  driver.init();
  driver1.voltage_power_supply = 7.6;
  driver1.init();
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  motor.foc_modulation = motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = motor1.controller = MotionControlType::velocity;
  motor.PID_velocity.P = motor1.PID_velocity.P = 0.2;
  motor.PID_velocity.I = motor1.PID_velocity.I = 10;
  motor.voltage_limit = motor1.voltage_limit = 7.6;
  motor.LPF_velocity.Tf = motor1.LPF_velocity.Tf = 0.01;
  motor.velocity_limit = motor1.velocity_limit = 50;
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();
}

void initServer() {
  // 设置HTTP路由
  server.on("/velocity0", HTTP_GET, []() {
    if (server.hasArg("value")) {
      motor.target = server.arg("value").toFloat();
      server.send(200, "text/plain", "Motor 0 velocity updated");
    } else {
      server.send(400, "text/plain", "Bad request for Motor 0");
    }
  });
  server.on("/speed0", HTTP_GET, []() {
    server.send(400, "text/plain", String(motor.shaft_velocity));
  });

  server.on("/velocity1", HTTP_GET, []() {
    if (server.hasArg("value")) {
      motor1.target = server.arg("value").toFloat();
      server.send(200, "text/plain", "Motor 1 velocity updated");
    } else {
      server.send(400, "text/plain", "Bad request for Motor 1");
    }
  });
  server.on("/speed1", HTTP_GET, []() {
    server.send(400, "text/plain", String(motor1.shaft_velocity));
  });
}