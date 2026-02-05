// On click, this popup will show the amount of battery life left - J

let batteryLife = 100;

export function BatteryPopup(){
    return(
        <div className="battery-popup">
            <p>The battery is at {batteryLife}%</p>
        </div>
    )
}