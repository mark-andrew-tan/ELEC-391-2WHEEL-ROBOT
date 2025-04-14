import 'package:flutter/material.dart';

class PowerButton extends StatefulWidget {
  final void Function(String command) onCommand;  // Callback to send commands

  const PowerButton({super.key, required this.onCommand});

  @override
  State<PowerButton> createState() => _PowerButtonState();
}

class _PowerButtonState extends State<PowerButton> {
  bool isOn = false;

  void _handleTapDown(TapDownDetails details) {
    setState(() {
      isOn = !isOn;
    });
    widget.onCommand('5');  // Send "5" on press
  }

  void _handleTapUp(TapUpDetails details) {
    widget.onCommand('0');  // Send "0" on release
  }

  @override
  Widget build(BuildContext context) {
    return GestureDetector(
      onTapDown: _handleTapDown,
      onTapUp: _handleTapUp,
      child: Icon(
        Icons.power_settings_new,
        color: isOn ? Colors.green : Colors.red,
        size: 50,
      ),
    );
  }
}