// On click, this popup will show the amount of battery life left - J

let batteryLife = 100;

// Calculate time remaining

let batteryTimeLeft = Math.trunc((batteryLife / 25) * 3600);
let batteryTimeHrs = Math.trunc(batteryTimeLeft / 3600);
let batteryTimeMins = Math.trunc(batteryTimeLeft % 3600 / 60);

export function BatteryPopup(){
    return(
        <div className="battery-popup">
            <p>The battery is at {batteryLife}%</p>
            <p>Time until battery dry: {batteryTimeHrs} hrs {batteryTimeMins} min</p>
        </div>
    )
}