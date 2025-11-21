"use client"

import { HashRouter as Router, Routes, Route } from 'react-router-dom'
import { Home } from "./pages/homepage"
import Science from "./pages/science"
import Gyroscope from './pages/gyroscope'


function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Home/>}/>
        <Route path="/science-payload" element={<Science/>}/>
        <Route path="/gyroscope" element={<Gyroscope/>}/>
      </Routes>
    </Router>

)
}

export default App