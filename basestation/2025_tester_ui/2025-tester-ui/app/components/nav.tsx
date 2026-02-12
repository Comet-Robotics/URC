// The Navbar Component

import Link from "next/link";
import { IconButton } from '@mui/material';
import NotificationsIcon from '@mui/icons-material/Notifications';
import Battery90Icon from '@mui/icons-material/Battery90';
import NetworkWifiIcon from '@mui/icons-material/NetworkWifi';

// Other components needed - J

import { Notifications } from '../components/notifications';
import { BatteryPopup } from '../components/batteryPopup';
import {WifiPopup} from "../components/wifiPopup";
import { useState } from 'react';

// icons documentation: https://mui.com/material-ui/icons/
// The function that renders the Navbar

export function Nav() {

  const [isNotifs, setIsNotifs] = useState(false);
  const [isBatPop, setIsBatPop] = useState(false);
  const [isWifiPop, setIsWifiPop] = useState(false);

  return (
    <nav>
        {/* Navigation links */}
        <div className="nav-links">
          <Link href="/" className={"navItems"} >Home</Link>
          <Link href="/science" className={"navItems"}>Science Payload</Link>
          <Link href="/gyroscope" className={"navItems"}>Gyroscope</Link>
        </div>
        
        <div className="notifs">
          <div className="notifications-icon-container">
            <IconButton aria-label="notifs" onClick={() => setIsNotifs(!isNotifs)}><NotificationsIcon /></IconButton>
          </div>
          
          <div className="utility-icons-container">
            <div className="icon-button-container">
              <IconButton aria-label="battery" onClick={() => setIsBatPop(!isBatPop)}><Battery90Icon /></IconButton>
            </div>
            <div className="icon-button-container">
              <IconButton aria-label="connection" onClick={() => setIsWifiPop(!isWifiPop)}><NetworkWifiIcon /></IconButton>
            </div>
          </div>
          
          {(isBatPop || isWifiPop) && (
            <div className="utility-popups">
              {isBatPop && <BatteryPopup />}
              {isWifiPop && <WifiPopup />}
            </div>
          )}
        </div>

        {isNotifs && <Notifications />}
    </nav>
  );
}   