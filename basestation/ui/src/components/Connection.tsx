import { useState, useEffect, useCallback } from 'react';
import {
    Card,
    CardContent,
    CardDescription,
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
    roverAddress: string | undefined;
}

const Connection = ({ roverAddress }: ControllerProps) => {
   
    
    return (
        <Card className="h-full w-full col-span-1 row-span-1">
            <CardHeader>
                <CardTitle>Connection Status</CardTitle>
                <CardDescription>{roverAddress ?? "Not Connected"}</CardDescription>
            </CardHeader>
            <CardContent>
               

            </CardContent>
            <CardFooter />
        </Card>
    );
};

export default Connection;
