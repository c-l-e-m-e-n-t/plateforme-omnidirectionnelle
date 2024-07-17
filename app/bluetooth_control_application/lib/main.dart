import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_blue/flutter_blue.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'dart:convert';

void main() {
  runApp(MyApp());
}

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: BluetoothApp(),
    );
  }
}

class BluetoothApp extends StatefulWidget {
  @override
  _BluetoothAppState createState() => _BluetoothAppState();
}

class _BluetoothAppState extends State<BluetoothApp> {
  FlutterBlue flutterBlue = FlutterBlue.instance;
  BluetoothDevice? device;

  double leftJoystickX = 0;
  double leftJoystickY = 0;
  double rightJoystickX = 0;
  double rightJoystickY = 0;

  @override
  void initState() {
    super.initState();
    flutterBlue.startScan(timeout: Duration(seconds: 4));

    flutterBlue.scanResults.listen((results) {
      for (ScanResult r in results) {
        if (r.device.name == "ESP32Robot") {
          device = r.device;
          flutterBlue.stopScan();
          _connectToDevice();
          break;
        }
      }
    });

    // Forcer l'orientation paysage
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
  }

  void _connectToDevice() async {
    if (device != null) {
      await device!.connect();
      print("Connected to device");
    }
  }

  void _sendCommand(String command) async {
    if (device != null) {
      var services = await device!.discoverServices();
      var uartService = services.firstWhere(
          (s) => s.uuid.toString() == "0000ffe0-0000-1000-8000-00805f9b34fb");
      var txChar = uartService.characteristics.firstWhere(
          (c) => c.uuid.toString() == "0000ffe1-0000-1000-8000-00805f9b34fb");
      await txChar.write(utf8.encode(command));
    }
  }

  void _onLeftJoystickChange(double x, double y) {
    setState(() {
      leftJoystickX = x;
      leftJoystickY = y;
    });
    _updateMovement();
  }

  void _onRightJoystickChange(double x, double y) {
    setState(() {
      rightJoystickX = x;
      rightJoystickY = y;
    });
    _updateMovement();
  }

  void _updateMovement() {
    // Combiner les valeurs des deux joysticks pour créer une commande
    String command =
        "${leftJoystickX.toStringAsFixed(2)},${leftJoystickY.toStringAsFixed(2)},${rightJoystickX.toStringAsFixed(2)},${rightJoystickY.toStringAsFixed(2)}";
    _sendCommand(command);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text("Omnidirectional Robot Controller"),
      ),
      body: Stack(
        children: <Widget>[
          Positioned(
            bottom: 16,
            left: 16,
            child: Joystick(
              mode: JoystickMode.all,
              listener: (details) {
                _onLeftJoystickChange(details.x, details.y);
              },
            ),
          ),
          Positioned(
            bottom: 16,
            right: 16,
            child: Joystick(
              mode: JoystickMode.all,
              listener: (details) {
                _onRightJoystickChange(details.x, details.y);
              },
            ),
          ),
        ],
      ),
    );
  }

  @override
  void dispose() {
    super.dispose();
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
    ]);
  }
}
