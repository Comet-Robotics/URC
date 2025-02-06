
import {
    Card,
    CardContent,

    CardFooter,
    CardHeader,
    CardTitle,
  } from "@/components/ui/card"
import { Vector3 } from "../../../../rover-msgs/bindings/Vector3";

const Telemetry = ({gps}:{gps: Vector3}) => {


    return (
        <Card className="h-full w-24">
            <CardHeader>
              <CardTitle>Telemetry</CardTitle>
            </CardHeader>
            <CardContent>
              <p>X: {gps.x} Y: {gps.y} Z: {gps.z}</p>
             
            </CardContent>
            <CardFooter>
      
            </CardFooter>
        </Card>
    );
};

export default Telemetry;
