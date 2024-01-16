import React, { Component } from "react";
import { getBuses } from "../services/busService";

class Track extends Component {
  state = {
    busses: [],
  };

  async componentDidMount() {
    try {
      const { data } = await getBuses();
      this.setState({ busses: data.registeredVehicles });
    } catch (error) {
      console.error("Error fetching users:", error.message);
    }
  }

  render({ busses } = this.state) {
    return (
      <div class="row row-cols-1 row-cols-md-3 g-4">
        {busses.map((bus) => (
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">Tracking {bus.vehicleID}</h5>
              <a href="/LocationTracking" class="btn btn-primary">
                Track
              </a>
            </div>
          </div>
        ))}
      </div>
    );
  }
}

export default Track;
