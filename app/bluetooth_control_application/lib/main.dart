import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:flutter_bluetooth_serial/flutter_bluetooth_serial.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:flutter_joystick/flutter_joystick.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Omni Platform Control',
      theme: ThemeData(primarySwatch: Colors.blue),
      home: HomeScreen(),
    );
  }
}

class HomeScreen extends StatefulWidget {
  @override
  _HomeScreenState createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  BluetoothState _bluetoothState = BluetoothState.UNKNOWN;
  BluetoothConnection? _connection;
  List<BluetoothDevice> _devicesList = [];
  bool _isConnecting = false;
  bool _isConnected = false;

  double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
  Timer? _joystickTimer;

  @override
  void initState() {
    super.initState();
    _initBluetooth();
    _startJoystickTimer();
  }

   void _startJoystickTimer() {
    _joystickTimer?.cancel();
    _joystickTimer = Timer.periodic(Duration(milliseconds: 100), (_) {
      _sendJoystickValues();
    });
  }

  void _sendJoystickValues() {
    if (_connection != null && _connection!.isConnected) {
      String command =
          "${x1.toInt()} ${y1.toInt()} ${x2.toInt()} ${y2.toInt()}";
      _connection!.output.add(Uint8List.fromList(utf8.encode(command + '\n')));
    }
  }

  Future<void> _initBluetooth() async {
    await _requestPermissions();
    await _initBluetoothState();
    await _getPairedDevices();
  }

  Future<void> _requestPermissions() async {
    await [
      Permission.bluetooth,
      Permission.bluetoothScan,
      Permission.bluetoothConnect,
      Permission.location,
    ].request();
  }

  Future<void> _initBluetoothState() async {
    _bluetoothState = await FlutterBluetoothSerial.instance.state;
    FlutterBluetoothSerial.instance
        .onStateChanged()
        .listen((BluetoothState state) {
      setState(() {
        _bluetoothState = state;
      });
    });
  }

  Future<void> _getPairedDevices() async {
    try {
      _devicesList = await FlutterBluetoothSerial.instance.getBondedDevices();
    } catch (e) {
      print('Error getting bonded devices: $e');
    }
    setState(() {});
  }

  void _showDeviceSelectionDialog() {
    showDialog(
      context: context,
      builder: (context) => AlertDialog(
        title: Text("Select Device"),
        content: SizedBox(
          width: double.maxFinite,
          child: ListView.builder(
            shrinkWrap: true,
            itemCount: _devicesList.length,
            itemBuilder: (context, index) => ListTile(
              title: Text(_devicesList[index].name ?? "Unknown device"),
              subtitle: Text(_devicesList[index].address),
              onTap: () {
                Navigator.of(context).pop();
                _connectToDevice(_devicesList[index]);
              },
            ),
          ),
        ),
      ),
    );
  }

  Future<void> _connectToDevice(BluetoothDevice device) async {
    setState(() {
      _isConnecting = true;
    });

    try {
      _connection = await BluetoothConnection.toAddress(device.address);
      print('Connected to ${device.name}');

      setState(() {
        _isConnected = true;
        _isConnecting = false;
      });

      _connection!.input!.listen(_onDataReceived).onDone(() {
        _onDisconnected();
      });
    } catch (e) {
      print('Error connecting to device: $e');
      setState(() {
        _isConnecting = false;
      });
    }
  }

  void _onDataReceived(Uint8List data) {
    print('Data received: ${ascii.decode(data)}');
  }

  void _onDisconnected() {
    print('Disconnected by remote request');
    setState(() {
      _isConnected = false;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text("Omni Platform Control")),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            _buildConnectionStatus(),
            SizedBox(height: 20),
            _buildConnectionButton(),
            SizedBox(height: 40),
            _buildJoysticks(),
          ],
        ),
      ),
    );
  }

  Widget _buildConnectionStatus() {
    if (_isConnecting) {
      return Text("Connecting...");
    } else if (_isConnected) {
      return Text("Connected", style: TextStyle(color: Colors.green));
    } else {
      return Text("Disconnected", style: TextStyle(color: Colors.red));
    }
  }

  Widget _buildConnectionButton() {
    return ElevatedButton(
      onPressed: _isConnected ? _disconnect : _showDeviceSelectionDialog,
      child: Text(_isConnected ? "Disconnect" : "Connect"),
    );
  }

  Widget _buildJoysticks() {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
      children: [
        _buildJoystick(
          onUpdate: (StickDragDetails details) {
            setState(() {
              x1 = details.x * 255;
              y1 = -details.y * 255; // Inverser l'axe Y
            });
          },
        ),
        _buildJoystick(
          onUpdate: (StickDragDetails details) {
            setState(() {
              x2 = details.x * 255;
              y2 = -details.y * 255; // Inverser l'axe Y
            });
          },
        ),
      ],
    );
  }

  Widget _buildJoystick({required Function(StickDragDetails) onUpdate}) {
    return Joystick(
      mode: JoystickMode.all,
      period: Duration(milliseconds: 100),
      listener: onUpdate,
    );
  }

  @override
  void dispose() {
    _joystickTimer?.cancel();
    _connection?.dispose();
    super.dispose();
  }


  void _disconnect() {
    _connection?.dispose();
    setState(() {
      _isConnected = false;
    });
  }
}
