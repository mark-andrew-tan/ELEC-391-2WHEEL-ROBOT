import 'dart:convert';
import 'dart:async';
import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'power_button.dart'; // Import the powerbutton class
import 'mute_button.dart'; // Import the mutebutton class

const String serviceUUID = "00000000-5EC4-4083-81CD-A10B8D5CF6EC";
const String characteristicUUID = "00000001-5EC4-4083-81CD-A10B8D5CF6EC";

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});
  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final _ble = FlutterReactiveBle();

  StreamSubscription<DiscoveredDevice>? _scanSub;
  StreamSubscription<ConnectionStateUpdate>? _connectSub;
  StreamSubscription<List<int>>? _notifySub;

  List<DiscoveredDevice> _devices = [];
  String? _selectedDeviceId;
  String? _selectedDeviceName;
  var _stateMessage = 'Scanning...';
  QualifiedCharacteristic? _writeCharacteristic;

  bool _isConnected = false;
  bool _isConnecting = false;

  Timer? _connectionTimeoutTimer;
  Timer? _commandTimer;
  String _currentCommand = '0';

  @override
  void initState() {
    super.initState();
    _scanSub = _ble.scanForDevices(withServices: []).listen(_onScanUpdate);
  }

  @override
  void dispose() {
    _notifySub?.cancel();
    _connectSub?.cancel();
    _scanSub?.cancel();
    _commandTimer?.cancel();
    _connectionTimeoutTimer?.cancel();
    super.dispose();
  }

  void _onScanUpdate(DiscoveredDevice d) {
    if (d.name.contains("BLE") &&
        !_devices.any((device) => device.id == d.id)) {
      setState(() {
        _devices.add(d);
      });
    }
  }

  void _connectToDevice() {
    if (_selectedDeviceId != null) {
      setState(() {
        _stateMessage = 'Connecting to $_selectedDeviceName...';
        _isConnecting = true;
      });

      _connectionTimeoutTimer = Timer(const Duration(seconds: 5), () {
        if (!_isConnected) {
          _disconnectFromDevice();
          setState(() {
            _stateMessage = 'Connection timed out.';
            _isConnecting = false;
          });
        }
      });

      _connectSub = _ble.connectToDevice(id: _selectedDeviceId!).listen(
        (update) {
          if (update.connectionState == DeviceConnectionState.connected) {
            setState(() {
              _stateMessage = 'Connected to $_selectedDeviceName!';
              _isConnected = true;
              _isConnecting = false;
            });
            _connectionTimeoutTimer?.cancel();
            _onConnected(_selectedDeviceId!);
          }
        },
        onError: (error) {
          setState(() {
            _stateMessage = 'Connection failed: $error';
            _isConnecting = false;
          });
          _connectionTimeoutTimer?.cancel();
        },
      );
    }
  }

  void _disconnectFromDevice() {
    try {
      _notifySub?.cancel();
      _connectSub?.cancel();
      _connectionTimeoutTimer?.cancel();
      setState(() {
        _isConnected = false;
        _stateMessage = 'Disconnected from $_selectedDeviceName.';
        _writeCharacteristic = null;
      });
    } catch (e) {
      setState(() {
        _stateMessage = 'Error during disconnection: $e';
      });
    }
  }

  void _onConnected(String deviceId) {
    final characteristic = QualifiedCharacteristic(
      deviceId: deviceId,
      serviceId: Uuid.parse(serviceUUID),
      characteristicId: Uuid.parse(characteristicUUID),
    );

    _writeCharacteristic = characteristic;

    _notifySub = _ble.subscribeToCharacteristic(characteristic).listen((bytes) {
      setState(() {
        _stateMessage = 'Data received: ${Utf8Decoder().convert(bytes)}';
      });
    });
  }

  Future<void> _sendCommand(String command) async {
    if (_currentCommand != command && _writeCharacteristic != null) {
      _currentCommand = command;
      try {
        await _ble.writeCharacteristicWithResponse(
          _writeCharacteristic!,
          value: utf8.encode(command),
        );
        setState(() {
          _stateMessage = "Command '$command' sent!";
        });
      } catch (e) {
        setState(() {
          _stateMessage = "Error sending command: $e";
        });
      }
    }
  }

  void _startHoldCommand(String command) {
    _sendCommand(command);  // Send immediately on press

    _commandTimer?.cancel();
    _commandTimer = Timer.periodic(const Duration(milliseconds: 150), (timer) {
      _sendCommand(command);
    });
  }

  void _stopHoldCommand() {
    _commandTimer?.cancel();
    _sendCommand('0');  // Send brake command on release
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.grey[100],
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Text(widget.title),
      ),
      body: Column(
        children: [
          Container(
            padding: const EdgeInsets.all(16.0),
            color: Colors.grey[300],
            width: double.infinity,
            child: _isConnecting
                ? const Center(child: CircularProgressIndicator())
                : Text(
                    _stateMessage,
                    style: Theme.of(context).textTheme.titleMedium,
                    textAlign: TextAlign.center,
                  ),
          ),
          if (_devices.isNotEmpty)
            Padding(
              padding: const EdgeInsets.all(16.0),
              child: DropdownButton<String>(
                isExpanded: true,
                hint: const Text("Select a BLE Device"),
                value: _selectedDeviceId,
                items: _devices.map((device) {
                  return DropdownMenuItem(
                    value: device.id,
                    child: Text(device.name),
                  );
                }).toList(),
                onChanged: (value) {
                  setState(() {
                    _selectedDeviceId = value;
                    _selectedDeviceName = _devices
                        .firstWhere((device) => device.id == value)
                        .name;
                  });
                },
              ),
            ),
          Padding(
            padding: const EdgeInsets.symmetric(horizontal: 16.0),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                const SizedBox(width: 50), // placeholder to balance layout
                Expanded(
                  child: Center(
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        backgroundColor: _isConnected ? Colors.redAccent : Colors.blueAccent,
                      ),
                      onPressed: _selectedDeviceId != null
                          ? (_isConnected ? _disconnectFromDevice : _connectToDevice)
                          : null,
                      child: Text(_isConnected ? 'Disconnect' : 'Connect'),
                    ),
                  ),
                ),
                MuteButton(
                  onCommand: (command) => _sendCommand(command),
                ),
              ],
            ),
          ),
          Expanded(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                GestureDetector(
                  onTapDown: (_) => _startHoldCommand('1'),  // Forward
                  onTapUp: (_) => _stopHoldCommand(),
                  child: _controlButton(Icons.arrow_upward, Colors.green),
                ),
                const SizedBox(height: 20),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    GestureDetector(
                      onTapDown: (_) => _startHoldCommand('3'),  // Left
                      onTapUp: (_) => _stopHoldCommand(),
                      child: _controlButton(Icons.arrow_back, Colors.blue),
                    ),
                    const SizedBox(width: 20),
                    GestureDetector(
                      onTapDown: (_) => _startHoldCommand('2'),  // Backward
                      onTapUp: (_) => _stopHoldCommand(),
                      child: _controlButton(Icons.arrow_downward, Colors.orange),
                    ),
                    const SizedBox(width: 20),
                    GestureDetector(
                      onTapDown: (_) => _startHoldCommand('4'),  // Right
                      onTapUp: (_) => _stopHoldCommand(),
                      child: _controlButton(Icons.arrow_forward, Colors.blue),
                    ),
                  ],
                ),
                const SizedBox(height: 40),
                PowerButton(
                  onCommand: (command) => _sendCommand(command),
                ),
                const SizedBox(height: 30),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    GestureDetector(
                      onTapDown: (_) => _startHoldCommand('8'), // Close
                      onTapUp: (_) => _stopHoldCommand(),
                      child: Container(
                        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
                        decoration: BoxDecoration(
                          color: Colors.purple,
                          borderRadius: BorderRadius.circular(12),
                        ),
                        child: const Text("Close", style: TextStyle(fontSize: 18, color: Colors.white)),
                      ),
                    ),
                    const SizedBox(width: 20),
                    GestureDetector(
                      onTapDown: (_) => _startHoldCommand('7'), // Open
                      onTapUp: (_) => _stopHoldCommand(),
                      child: Container(
                        padding: const EdgeInsets.symmetric(horizontal: 24, vertical: 16),
                        decoration: BoxDecoration(
                          color: Colors.teal,
                          borderRadius: BorderRadius.circular(12),
                        ),
                        child: const Text("Open", style: TextStyle(fontSize: 18, color: Colors.white)),
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 30),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    GestureDetector(
                      onTapDown: (_) => _startHoldCommand('a'), // Previous
                      onTapUp: (_) => _stopHoldCommand(),
                      child: Icon(
                        Icons.skip_previous,
                        size: 50, // Make this larger
                        color: Colors.grey,  // Same color for both
                      ),
                    ),
                    const SizedBox(width: 20),
                    GestureDetector(
                      onTapDown: (_) => _startHoldCommand('b'), // Skip
                      onTapUp: (_) => _stopHoldCommand(),
                      child: Icon(
                        Icons.skip_next,
                        size: 50, // Make this larger
                        color: Colors.grey,  // Same color for both
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _controlButton(IconData icon, Color color) {
    return IconButton(
      icon: Icon(icon, size: 50, color: Colors.white),
      style: ButtonStyle(
        backgroundColor: MaterialStateProperty.all(color),
        padding: MaterialStateProperty.all(const EdgeInsets.all(16)),
      ),
      onPressed: null,
    );
  }
}