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
    const [gamePadDetected,setGamePadDetected] = useState(false); 
    const updateMovement = useCallback((gamepad: Gamepad) => {
        const linearX = gamepad.axes[1];
        const angularZ = gamepad.axes[0];
        const newTwist: Twist = {
            linear: { x: linearX * linearSpeed, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angularZ * angularSpeed },
        };

        if (!twist ||  Math.abs(newTwist.linear!.x! - twist.linear!.x!) > 0.01 || Math.abs(newTwist.angular!.z! - twist.angular!.z! ) > 0.01) {
            setTwist(newTwist);
        }
        
    }, [linearSpeed, angularSpeed]);

    const handleGamepadInput = useCallback(() => {
        const gamepads = navigator.getGamepads();
        const gamepad = gamepads[0];
        if (gamepad){
            setGamePadDetected(true);
        }else{
            setGamePadDetected(false);
        }
        if ( gamepad && sending) {
            console.log(gamepad)
            updateMovement(gamepad);
        } 
    }, [ updateMovement,sending]);

    useEffect(() => {
        let intervalId: NodeJS.Timeout | undefined;

        
        intervalId = setInterval(handleGamepadInput, 200);
        

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
    
    return (
        <Card className="h-full w-full col-span-1 row-span-1">
            <CardHeader className='flex justify-between flex-row'>
                <CardTitle>Controller Control</CardTitle>
                <Switch id="sending" checked={sending}  
                      onCheckedChange={(a ) => {console.log(a); setSending(a);}} />
                  
            </CardHeader>
            <CardContent>
                <p>{gamePadDetected ? "Controller Detected" : "Controller Disconnected"}</p>
                <p>Twist: {twist && JSON.stringify(twist)}</p>
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
                    <Button onClick={()=> sendMovement({
                    linear: { x: 1.0, y: 0, z: 0 },
                    angular: { x: 0, y: 1.0, z: 0 }
                })}>Test</Button>
                </div>

            </CardContent>
            <CardFooter />
        </Card>
    );
};

export default Controller;
