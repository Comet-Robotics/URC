
import {
    Card,
    CardContent,

    CardFooter,
    CardHeader,
    CardTitle,
  } from "@/components/ui/card"
import { Map } from "./Map";
import {IMUDisplay} from "./IMUDisplay";
import { GPSData, IMUData } from "@/types/binding";

const Telemetry = ({gps,imu}:{gps?: GPSData,imu?: IMUData}) => {


    return (
        <Card className="h-full w-full col-span-2">
            <CardHeader>
              <CardTitle>Telemetry</CardTitle>
            </CardHeader>
            <CardContent className="flex flex-row flex-wrap gap-4">
              <Map roverPosition={ gps ?[gps.latitude!,gps.longitude!] : undefined}/>           
              <IMUDisplay imu={imu} />
            </CardContent>
            <CardFooter>
      
            </CardFooter>
        </Card>
    );
};

export default Telemetry;
