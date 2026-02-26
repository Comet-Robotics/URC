// The footer for ALL pages - J

let hrs = 0;
let mins = 0;
let secs = 0;
let hrsString, minsString, secsString;
let currentMission = "No Mission Selected";

export function Footer(){

    // Keeps the numbers in the double digits, even if the number is only one digit - J
    // Hours

    if(hrs <= 9){
        hrsString = "0" + hrs;
    } else{
        hrsString = hrs;
    }

    // Minutes

    if(mins <= 9){
        minsString = "0" + mins;
    } else{
        minsString = mins;
    }

    // Seconds

    if(secs <= 9){
        secsString = "0" + secs;
    } else{
        secsString = secs;
    }

    // The actual footer itself - J

    return(
        <div className = "footer">
            <p className="footer-text"><strong>Current Mission:</strong> {currentMission}</p>
            <p className="footer-text"><strong>Time Remaining:</strong> {hrsString}:{minsString}:{secsString}</p>
        </div>
    )
}