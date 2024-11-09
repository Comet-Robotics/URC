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
        const linearX = gamepad.axes[1];
        const angularZ = gamepad.axes[0];

        setLinear({ x: Math.round(-linearX * linearSpeed * 100) / 100, y: 0, z: 0 });
        setAngular({ x: 0, y: 0, z: Math.round(angularZ * angularSpeed * 100) / 100 });
    };

    const sendMovement = (linearState: any, angularState:any) => {
        console.log("Sending movement", linearState, angularState);
        fetch('/rover/twist', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ linear: linearState, angular: angularState }),
        });
    };

    const handleGamepadInput = () => {
        const gamepads = navigator.getGamepads();
        const gamepad = gamepads[0];

        if (controlMode === "gamepad" && gamepad) {
            updateMovement(gamepad);
            sendMovement(linear, angular);
            setSending(true);
        }
    };

    const handleKeyDown = (event: KeyboardEvent) => {
        if (controlMode !== "keyboard") {
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
            sendMovement(newLinear, newAngular);
        } else {
            setSending(false);
        }
    };

    useEffect(() => {
        window.addEventListener('keydown', handleKeyDown);

        let intervalId: NodeJS.Timeout;

        if (controlMode === "gamepad") {
            intervalId = setInterval(handleGamepadInput, 20);
        }

        return () => {
            clearInterval(intervalId);
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [controlMode]);

    useEffect(() => {
        if (sending) {
            sendMovement(linear, angular);
        }
    }, [linear, angular, sending]);

    return (
        <Card className="h-full w-full">
            <CardHeader>
                <CardTitle>Manual Control</CardTitle>
                <Select defaultValue='keyboard' onValueChange={(a) => {
                    console.log("changing " + a);
                    if (a !== undefined) {
                        setMode(a);
                        if (a === "keyboard") {
                            setLinear({ x: 0, y: 0, z: 0 });
                            setAngular({ x: 0, y: 0, z: 0 });
                        }
                    }
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
                <p>Sending: {sending ? "Yes" : "No"} </p>
                <p>Angular Speed {angularSpeed}</p>
                <Slider defaultValue={[8]} max={10} step={0.5} onValueChange={(a) => setAngularSpeed(a[0])} />
                <p>Linear Speed {linearSpeed}</p>
                <Slider defaultValue={[2]} max={8} step={0.5} onValueChange={(a) => setLinearSpeed(a[0])} />
            </CardContent>
            <CardFooter>
            </CardFooter>
        </Card>
    );
};

export default Controller;
