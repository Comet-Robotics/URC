import { set } from 'astro:schema';
import React, { useState, useEffect } from 'react';
import { Slider } from './ui/slider';

const Controller = () => {
    const [linear, setLinear] = useState({ x: 0, y: 0, z: 0 });
    const [angular, setAngular] = useState({ x: 0, y: 0, z: 0 });
    const [linearSpeed, setLinearSpeed] = useState(3);
    const [angularSpeed, setAngularSpeed] = useState(8);
    const [sending, setSending] = useState(false);
    const updateMovement = (gamepad: Gamepad) => {
        // Assume the left stick controls linear x (forward/backward) movement
        const linearX = gamepad.axes[1];  // typically, left stick Y axis
        // Assume the right stick controls angular z (rotation) movement
        const angularZ = gamepad.axes[0]; // typically, right stick X axis

        // Map joystick values to desired speed ranges
        setLinear({ x: -linearX * linearSpeed, y: 0, z: 0 });  // Adjust scale as needed
        setAngular({ x: 0, y: 0, z: angularZ * angularSpeed }); // Adjust scale as needed
    };

    const sendMovement = () => {
        fetch('/rover/twist', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ linear, angular }),
        });
    };

    useEffect(() => {
        const handleGamepadInput = () => {
            const gamepads = navigator.getGamepads();
            const gamepad = gamepads[0]; // Use the first connected gamepad

            if (gamepad) {
                updateMovement(gamepad);
                if (gamepad.buttons[0].pressed){
                    setSending(true);

                    sendMovement();

                }else{
                    setSending(false);
                }
            }
        };

        // Set up a polling interval for gamepad input
        const intervalId = setInterval(handleGamepadInput, 100); // Poll every 100 ms

        return () => clearInterval(intervalId); // Clean up on component unmount
    }, [linear, angular]);

    return (
        <div>
            <p>Linear: {JSON.stringify(linear)}</p>
            <p>Angular: {JSON.stringify(angular)}</p>
            <p>Sending: {sending ? "Yes": "No"} </p>
            <p>Angular Speed {angularSpeed}</p>
            <Slider defaultValue={[8]} max={10} step={0.5} onValueChange={(a) => setAngularSpeed(a[0])} />       
            <p>Linear Speed {linearSpeed}</p>
            <Slider defaultValue={[2]} max={8} step={0.5} onValueChange={(a) => setLinearSpeed(a[0])}  />       
             </div>
    );
};

export default Controller;
