
import {
    Card,
    CardContent,

    CardFooter,
    CardHeader,
    CardTitle,
  } from "@/components/ui/card"

const Telemetry = ({messages}:{messages: any[]}) => {


    return (
        <Card className="h-full w-24">
            <CardHeader>
              <CardTitle>Telemetry</CardTitle>
            </CardHeader>
            <CardContent>
              {JSON.stringify(messages)}
             
            </CardContent>
            <CardFooter>
      
            </CardFooter>
        </Card>
    );
};

export default Telemetry;
