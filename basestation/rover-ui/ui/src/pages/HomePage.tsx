import React from "react";
import VideoStream from "../components/VideoStream";
import Telemetry from "../components/Telemetry";
import Controller from "../components/Controller";
import Control from "@/components/Control";
import {  useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket"
import { Message } from "@/lib/types";
import { Vector3 } from "../../../../rover-msgs/bindings/Vector3";
import { Map } from "@/components/Map";

export default function HomePage() {

  const [socketUrl, setSocketUrl] = useState('/message_stream');
  const [messageHistory, setMessageHistory] = useState<Message[]>([]);
  const [gps, setGPS ] = useState<Vector3>({x:0.0,y:0.0,z:0.0});

  const { sendMessage, lastMessage, readyState } = useWebSocket(socketUrl);

  useEffect(() => {
    if (lastMessage !== null) {
      // console.log(lastMessage)
      let message: Message = JSON.parse(lastMessage.data);
      if (message.type == "GPS"){
        setGPS(message)
      }
      setMessageHistory((prev) => prev.concat());
    }
  }, [lastMessage]);

  const connectionStatus = {
    [ReadyState.CONNECTING]: 'Connecting',
    [ReadyState.OPEN]: 'Open',
    [ReadyState.CLOSING]: 'Closing',
    [ReadyState.CLOSED]: 'Closed',
    [ReadyState.UNINSTANTIATED]: 'Uninstantiated',
  }[readyState];


  return (
    <section className="grid grid-cols-3 gap-4">
      <VideoStream/>

      <Controller sendMovement={(twist) => sendMessage(JSON.stringify(twist))} />
      <Telemetry gps={gps} />
      {/* <Map roverPosition={[gps.x,gps.y]}/> */}
    </section>
  );
}
