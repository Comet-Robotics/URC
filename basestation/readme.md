4 projects here

## Projects

### Fake Rover (fake-rover)
As the name states its meant to emulate the rover side so we can build away from rover.

### Rover Messages (rover-msgs)
Library that holds the messages we will send between rover and base
over tcp.

### Rover UI (rover-ui)
This is the rust webserver that holds the code for webrtc and tcp connection to the rover. the UI (ui) directory is where we keep the react + Vite project. 



## Running

### Dependicies
ffmpeg + rust + node + lots of other prolly or not. if you get error saying pkg missing install it


### Building UI
any change made to UI needs to be built to see it reflect on rust website
`npm run build` from the rover-ui/ui directory does the trick

### Running webserver
`cargo run` from rover-ui 

### Fake Rover
`cargo run ` from fake-rover after webserver is up.




