// This component is for the SCIENCE PAYLOAD page - J
// This component will get sensor data and display it

// Data inputted to show the sensor data

let temp = 70;
let humid = 50;

// Actual component shown

export function SensorData(){
    return(
        <div className = "dataBlock">
            <p className="dataText">Temperature: {temp} C</p>
            <p className="dataText">Humidity: {humid}%</p>
        </div>
    )
}