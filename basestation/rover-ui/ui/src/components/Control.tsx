
import {
    Card,
    CardContent,

    CardFooter,
    CardHeader,
    CardTitle,
  } from "@/components/ui/card"
import { Switch } from "@/components/ui/switch"
const Control = () => {


    return (
        <Card className="h-full w-full">
            <CardHeader>
              <CardTitle>Control</CardTitle>
            </CardHeader>
            <CardContent>
            <Switch />
            </CardContent>
            <CardFooter>
              
            </CardFooter>
        </Card>
    );
};

export default Control;
