import React from "react";
import VideoStream from "../components/VideoStream";
import Telemetry from "../components/Telemetry";
import Controller from "../components/Controller";
import Control from "@/components/Control";
import {  useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket"
import { Map } from "@/components/Map";
import { decodeMessage ,encodeTwist,GPSData,IMUData,Message,Quaternion,Vector3} from "@/types/binding";

export default function HomePage() {

  const [socketUrl, setSocketUrl] = useState('/message_stream');
  const [messageHistory, setMessageHistory] = useState<Message[]>([]);
  const [gps, setGPS ] = useState<GPSData>();
  const [imu, setIMU] = useState<IMUData>();

  const { sendMessage, lastMessage, readyState } = useWebSocket(socketUrl);

  useEffect(() => {
    const processMessage = async () => {
      if (lastMessage !== null) {
        console.log(lastMessage);
        
        // If lastMessage.data is a Blob
        if (lastMessage.data instanceof Blob) {
          const arrayBuffer = await lastMessage.data.arrayBuffer();
          const uint8Array = new Uint8Array(arrayBuffer);
          let message: Message = decodeMessage(uint8Array);
          if (message.gps) {
            setGPS(message.gps);
          }
          if (message.imu) {
            setIMU(message.imu);
          }
        } 
       
      }
    };
  
    processMessage();
  }, [lastMessage]);

  const connectionStatus = {
    [ReadyState.CONNECTING]: 'Connecting',
    [ReadyState.OPEN]: 'Open',
    [ReadyState.CLOSING]: 'Closing',
    [ReadyState.CLOSED]: 'Closed',
    [ReadyState.UNINSTANTIATED]: 'Uninstantiated',
  }[readyState];


  return (
    <section className="grid grid-cols-3 gap-4 p-4">
      <VideoStream/>

      <Controller sendMovement={(twist) => sendMessage(encodeTwist(twist))} />
      <Telemetry gps={gps} imu={imu}/>
    </section>
  );
}
