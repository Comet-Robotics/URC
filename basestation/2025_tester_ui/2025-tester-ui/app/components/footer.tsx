// The footer for ALL

export function Footer(){
    return(
        <div className = "footer">
            <p>Current Mission: [Insert Mission Here]</p>
            <p>Time Remaining: 
                <span id="hrs">00</span>
                :
                <span id="mins">00</span>
                :
                <span id="secs">00</span>
            </p>
            <div className="btnContain">
                <button className="btnStart">Start</button>
                <button className="btnPause">Stop</button>
                <button className="btnStop">Pause</button>
            </div>
        </div>
    )
}