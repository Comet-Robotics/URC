import { useState, useEffect } from 'react';
import {
    Card,
    CardContent,

    CardFooter,
    CardHeader,
    CardTitle,
  } from "@/components/ui/card"
  import {
    Select,
    SelectContent,
    SelectItem,
    SelectTrigger,
    SelectValue,
  } from "@/components/ui/select"
import { Slider } from './ui/slider';
  
const Controller = () => {
    const [linear, setLinear] = useState({ x: 0, y: 0, z: 0 });
    const [angular, setAngular] = useState({ x: 0, y: 0, z: 0 });
    const [controlMode, setMode] = useState("keyboard");
    const [linearSpeed, setLinearSpeed] = useState(3);
    const [angularSpeed, setAngularSpeed] = useState(8);
    const [sending, setSending] = useState(false);
    const updateMovement = (gamepad: Gamepad) => {
        // Assume the left stick controls linear x (forward/backward) movement
        const linearX = gamepad.axes[1];  // typically, left stick Y axis
        // Assume the right stick controls angular z (rotation) movement
        const angularZ = gamepad.axes[0]; // typically, right stick X axis

        // Map joystick values to desired speed ranges
        setLinear({ x: Math.fround(-linearX * linearSpeed), y: 0, z: 0 });  // Adjust scale as needed
        setAngular({ x: 0, y: 0, z: Math.fround(angularZ * angularSpeed)}); // Adjust scale as needed
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


            if (controlMode == "gamepad" && gamepad && gamepad.connected ) {
                let oldLinear = linear;
                let oldAngular = angular;

                updateMovement(gamepad);
                if (oldAngular !== angular || oldLinear !== linear) {
                    setSending(true);

                    sendMovement();

                }else{
                    setSending(false);
                }
            }
        };
        const handleKeyDown = (event: KeyboardEvent) => {
            if (controlMode != "keyboard"){
                return;
            }
            let newLinear = { ...linear };
            let newAngular = { ...angular };

            switch (event.key) {
                case 'w':
                    newLinear.x = linearSpeed;
                    break;
                case 's':
                    newLinear.x = -linearSpeed;
                    break;
                case 'a':
                    newAngular.z = angularSpeed;
                    break;
                case 'd':
                    newAngular.z = -angularSpeed;
                    break;
                default:
                    break;
            }

            if (newLinear.x !== linear.x || newAngular.z !== angular.z) {
                setLinear(newLinear);
                setAngular(newAngular);
                setSending(true);
                sendMovement();
            } else {
                setSending(false);
            }
        };

        window.addEventListener('keydown', handleKeyDown);

            
            
        // Set up a polling interval for gamepad input
        const intervalId = setInterval(handleGamepadInput, 100); // Poll every 100 ms

        return () => {
            clearInterval(intervalId);
                    window.removeEventListener('keydown', handleKeyDown);
                }; // Clean up on component unmount
    }, [linear, angular]);

    return (
        <Card className="h-full w-full">
             <CardHeader>
               <CardTitle>Manual Control</CardTitle>
               <Select defaultValue='keyboard' onValueChange={(a)=>{
                     setMode(a);
               }}>
                    <SelectTrigger className="w-[180px]">
                        <SelectValue placeholder="Control Surface" />
                    </SelectTrigger>
                    <SelectContent>
                        <SelectItem value="gamepad">Controller</SelectItem>
                        <SelectItem value="keyboard">Keyboard</SelectItem>
                    </SelectContent>
                </Select>

             </CardHeader>
             <CardContent>
                <p>Linear: {JSON.stringify(linear)}</p>
                <p>Angular: {JSON.stringify(angular)}</p>
                <p>Sending: {sending ? "Yes": "No"} </p>
                <p>Angular Speed {angularSpeed}</p>
                <Slider defaultValue={[8]} max={10} step={0.5} onValueChange={(a) => setAngularSpeed(a[0])} />       
                <p>Linear Speed {linearSpeed}</p>
                <Slider defaultValue={[2]} max={8} step={0.5} onValueChange={(a) => setLinearSpeed(a[0])}  />       
             </CardContent>
             <CardFooter>
               <p>Card Footer</p>
             </CardFooter>
        </Card>
    );
};

export default Controller;
