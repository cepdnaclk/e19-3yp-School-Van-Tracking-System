import React, { Component } from "react";
import { Route, Switch, Redirect } from "react-router-dom";
import NavBar from "./components/navBar";
import Track from "./components/Track";
import Bus from "./components/Bus";
import RegisterUser from "./components/RegisterUser";
import UserRecord from "./components/UserRecord";
import DriverRecord from "./components/DriverRecord";
import ExtraService from "./components/ExtraService";
import Home from "./components/home";
import NotFound from "./components/notFound";
import RegisterForm from "./components/driverRegisterForm";
import AmbulanceRegisterForm from "./components/mbulanceRegisterForm";
import AssignBusses from "./components/assignBusses";
import LocationTracking from "./components/LocationTracking";

class App extends Component {
  state = {};
  render() {
    return (
      <>
        <NavBar />
        <main className="container">
          <Switch>
            <Route path="/Track" component={Track} />
            <Route path="/Bus" component={Bus} />
            <Route path="/RegisterUser" component={RegisterUser} />
            <Route path="/UserRecord" component={UserRecord} />
            <Route path="/DriverRecord" component={DriverRecord} />
            <Route path="/ExtraService" component={ExtraService} />
            <Route path="/not-found" component={NotFound} />
            <Route path="/driverRegisterForm" component={RegisterForm} />
            <Route
              path="/ambulanceRegisterForm"
              component={AmbulanceRegisterForm}
            />
            <Route path="/assignBusses" component={AssignBusses} />
            <Route path="/LocationTracking" component={LocationTracking} />
            <Route path="/home" component={Home} />
            <Redirect from="/" exact to="/home" />
            <Redirect to="/not-found" />
          </Switch>
        </main>
      </>
    );
  }
}

export default App;
