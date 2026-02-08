// On click, this wifi popup will show up

let connection = 100;
let connectStatus;

function checkConnection(){
    let cStatus;

    if(connection < 25){
        cStatus = "weak";
    } else if(connection >= 25 && connection < 50){
        cStatus = "intermediate";
    } else if(connection >= 50 && connection < 75){
        cStatus = "medium";
    } else if (connection >= 75 && connection <= 100){
        cStatus = "strong";
    } else{
        cStatus = "invalid"
    }

    return cStatus;
}

export function WifiPopup(){
    connectStatus = checkConnection()

    return(
        <div className = "wifi-popup">
            <p>The connection status is {connectStatus}.</p>
        </div>
    )
}