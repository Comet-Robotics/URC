import {
    Card,
    CardContent,
    CardDescription,
    CardFooter,
    CardHeader,
    CardTitle,
} from "@/components/ui/card";
import React from "react";

// Function that displays a visual representation of the battery level - JC

const BatteryIndicator = ({ batteryLevel }: { batteryLevel: number | null }) => {

    // Variables needed for the battery indicator - JC

    // Determine the color based on battery level - JC

    const getBatteryColor = (level: number) => {
        if (level > 50) return "green";
        if (level > 20) return "yellow";
        return "red";
    };
}

// The actual battery component - JC
// Won't comment on the obvious parts, because if you don't know what the obviously named components are at this point, you have bigger problems - JC

const Battery = ({ batteryLevel }: { batteryLevel: number | null }) => {
    return (
        <Card className="h-full w-full col-span-1 row-span-1">
            <CardHeader>
                <CardTitle>Battery Level</CardTitle>
                <CardDescription>
                    {batteryLevel !== null ? `${batteryLevel}%` : "Unknown"}
                </CardDescription>
            </CardHeader>
            <CardContent>
                {/* Additional content can be added here if needed - JC */}
            </CardContent>
            <CardFooter />
        </Card>
    );
};

export default Battery;