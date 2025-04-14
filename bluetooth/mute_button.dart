import 'package:flutter/material.dart';

class MuteButton extends StatefulWidget {
  final void Function(String command) onCommand;  // Callback to send commands

  const MuteButton({super.key, required this.onCommand});

  @override
  State<MuteButton> createState() => _MuteButtonState();
}

class _MuteButtonState extends State<MuteButton> {
  bool isOn = false;

  void _handleTapDown(TapDownDetails details) {
    setState(() {
      isOn = !isOn;
    });
    widget.onCommand('9');  // Send "5" on press
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
      isOn ? Icons.volume_up : Icons.volume_off_sharp,
      color: isOn ? Colors.green : Colors.red,
      size: 50,
    ),
  );
 }
}