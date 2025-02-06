import React from "react";
import VideoStream from "../components/VideoStream";
import Telemetry from "../components/Telemetry";
import Controller from "../components/Controller";
import Control from "@/components/Control";
import {  useEffect, useState } from "react";
import useWebSocket, { ReadyState } from "react-use-websocket"
import { Message } from "@/lib/types";

export default function HomePage() {

  const [socketUrl, setSocketUrl] = useState('/message_stream');
  const [messageHistory, setMessageHistory] = useState<Message[]>([]);

  const { sendMessage, lastMessage, readyState } = useWebSocket(socketUrl);

  useEffect(() => {
    if (lastMessage !== null) {
      console.log(lastMessage)
      setMessageHistory((prev) => prev.concat(JSON.parse(lastMessage.data)));
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
    <section className="container flex flex-row justify-evenly gap-3 pb-8 pt-6 md:py-10">
     {connectionStatus} 
      <Controller sendMovement={(twist) => sendMessage(JSON.stringify(twist))} />
      <VideoStream/>
      <Telemetry messages={messageHistory.slice(-2)} />
    </section>
  );
}
