import React, { Component } from "react";
import one from "../images/one.jpg";

class Home extends Component {
  state = {};
  render() {
    return (
      <>
        <div class="card text-bg-dark mb-3" style={{ "max-height": 300 }}>
          <img
            src={one}
            class="card-img"
            alt="..."
            style={{ "max-height": 300 }}
          />
          <div class="card-img-overlay">
            {/* <h5 class="card-title">Card title</h5>
            <p class="card-text">
              This is a wider card with supporting text below as a natural
              lead-in to additional content. This content is a little bit
              longer.
            </p>
            <p class="card-text">
              <small>Last updated 3 mins ago</small>
            </p> */}
          </div>
        </div>
        <div class="row row-cols-1 row-cols-md-3 g-4">
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">Tracking Vehicles</h5>
              <p class="card-text">You can track all your busses here</p>
              <a href="/Track" class="btn btn-primary">
                Track
              </a>
            </div>
          </div>
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">Manage Vehicles</h5>
              <p class="card-text">You can manage busses here</p>
              <a href="/Bus" class="btn btn-primary">
                Busses
              </a>
            </div>
          </div>
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">Driver Registration</h5>
              <p class="card-text">You can register drivers here</p>
              <a href="/RegisterUser" class="btn btn-primary">
                Register
              </a>
            </div>
          </div>
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">User Records</h5>
              <p class="card-text">
                You can see records of all your students' here
              </p>
              <a href="/UserRecord" class="btn btn-primary">
                User Records
              </a>
            </div>
          </div>
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">Driver records</h5>
              <p class="card-text">
                You can see records of all your drivers' here
              </p>
              <a href="/DriverRecord" class="btn btn-primary">
                Driver Records
              </a>
            </div>
          </div>
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">Services</h5>
              <p class="card-text">You can alert ambulances from here</p>
              <a href="/ExtraService" class="btn btn-primary">
                Services
              </a>
            </div>
          </div>
        </div>
      </>
    );
  }
}

export default Home;
