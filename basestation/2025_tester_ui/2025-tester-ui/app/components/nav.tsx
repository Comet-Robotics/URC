// The Navbar Component

import { NavLink } from "react-router-dom";
import { IconButton } from '@mui/material';
import NotificationsIcon from '@mui/icons-material/Notifications';
import Battery90Icon from '@mui/icons-material/Battery90';
import NetworkWifiIcon from '@mui/icons-material/NetworkWifi';
import { Notifications } from '../components/notifications';
import { useState } from 'react';
// icons documentation: https://mui.com/material-ui/icons/

// The function that renders the Navbar

export function Nav() {

  const [isVisible, setIsVisible] = useState(false);

  return (
    <nav>

        {/* Navigation links */}

        <NavLink to="/" className={"navItems"} >Home</NavLink>
        <NavLink to="/science-payload" className={"navItems"}>Science Payload</NavLink>
        <NavLink to="/gyroscope" className={"navItems"}>Gyroscope</NavLink>
        <div className="notifs">
          <IconButton aria-label="notifs" onClick={() => setIsVisible(!isVisible)}><NotificationsIcon /></IconButton>
          <IconButton aria-label="battery"><Battery90Icon /></IconButton>
          <IconButton aria-label="connection"><NetworkWifiIcon /></IconButton>
        </div>

        {isVisible && <Notifications />}
    </nav>
  );
}   