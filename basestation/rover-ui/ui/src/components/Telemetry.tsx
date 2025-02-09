
import {
    Card,
    CardContent,

    CardFooter,
    CardHeader,
    CardTitle,
  } from "@/components/ui/card"
import { Vector3 } from "../../../../rover-msgs/bindings/Vector3";
import { Map } from "./Map";
import { IMU } from "../../../../rover-msgs/bindings/IMU";
import {IMUDisplay} from "./IMUDisplay";

const Telemetry = ({gps,imu}:{gps: Vector3,imu: IMU}) => {


    return (
        <Card className="h-full w-full col-span-2">
            <CardHeader>
              <CardTitle>Telemetry</CardTitle>
            </CardHeader>
            <CardContent className="flex flex-row flex-wrap gap-4">
              <Map roverPosition={[gps.x,gps.y]}/>           
              <IMUDisplay imu={imu} />
            </CardContent>
            <CardFooter>
      
            </CardFooter>
        </Card>
    );
};

export default Telemetry;
