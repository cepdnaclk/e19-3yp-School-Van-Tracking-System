import React, { Component } from "react";

class ExtraService extends Component {
  state = {};
  render() {
    return (
      <div class="row row-cols-1 row-cols-md-3 g-4">
        <div class="card">
          <div class="card-body">
            <h5 class="card-title">Alert to an ambulance service</h5>
            <p class="card-text">
              You can alert to an ambulance service from here
            </p>
            <a href="/driverRegisterForm" class="btn btn-primary">
              Alert
            </a>
          </div>
        </div>
      </div>
    );
  }
}

export default ExtraService;
