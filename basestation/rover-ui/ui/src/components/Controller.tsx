import { useState, useEffect, useCallback } from 'react';
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
import { Twist } from '@/types/binding';
import { Button } from './ui/button';
import { Switch } from './ui/switch';
import { Label } from './ui/label';

interface ControllerProps {
    sendMovement: (msg: Twist) => void;
}

const Controller = ({ sendMovement }: ControllerProps) => {
    const [twist, setTwist] = useState<Twist>();
    const [linearSpeed, setLinearSpeed] = useState(2);
    const [angularSpeed, setAngularSpeed] = useState(8);
    const [sending, setSending] = useState(false);

    const updateMovement = useCallback((gamepad: Gamepad) => {
        const linearX = gamepad.axes[1];
        const angularZ = gamepad.axes[0];

        setTwist({
            linear: { x: Math.round(-linearX * linearSpeed * 100) / 100, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: Math.round(angularZ * angularSpeed * 100) / 100 },
        });
    }, [linearSpeed, angularSpeed]);

    const handleGamepadInput = useCallback(() => {
        const gamepads = navigator.getGamepads();
        const gamepad = gamepads[0];

        if ( gamepad && sending) {
            updateMovement(gamepad);
        } 
    }, [ updateMovement]);

    useEffect(() => {
        let intervalId: NodeJS.Timeout | undefined;

        
        intervalId = setInterval(handleGamepadInput, 20);
        

        return () => {
            if (intervalId) {
                clearInterval(intervalId);
            }
        };
    }, [handleGamepadInput]);

    useEffect(() => {
        if (sending && twist) {
            sendMovement(twist);
            setTwist(undefined);
        }
    }, [twist, sending, sendMovement]);
    console.log(sending)
    return (
        <Card className="h-full w-full col-span-1">
            <CardHeader>
                <CardTitle>Controller Control</CardTitle>
            </CardHeader>
            <CardContent>
         
                <p>Linear: {JSON.stringify(twist?.linear)}</p>
                <p>Angular: {JSON.stringify(twist?.angular)}</p>
                <Label htmlFor="sending">Enabled</Label>
                <Switch id="sending" checked={sending}  
                      onCheckedChange={(a ) => {console.log(a); setSending(a);}} />
                <div className="space-y-4">
                    <div>
                        <p>Angular Speed: {angularSpeed}</p>
                        <Slider 
                            defaultValue={[8]} 
                            max={10} 
                            step={0.5} 
                            onValueChange={(values) => setAngularSpeed(values[0])} 
                        />
                    </div>
                    <div>
                        <p>Linear Speed: {linearSpeed}</p>
                        <Slider 
                            defaultValue={[2]} 
                            max={8} 
                            step={0.5} 
                            onValueChange={(values) => setLinearSpeed(values[0])} 
                        />
                    </div>
                </div>

            </CardContent>
            <CardFooter />
        </Card>
    );
};

export default Controller;
