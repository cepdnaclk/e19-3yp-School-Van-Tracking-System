import React from "react";
import Joi from "joi-browser";
import Form from "./common/form";
import { addNewBus } from "../services/busService";
import { getBuses } from "../services/busService";

class Bus extends Form {
  state = {
    data: {
      vehicleID: "",
      School: "",
      seats: "",
    },
    errors: {},
    busses: [],
  };

  schema = {
    vehicleID: Joi.string().required().label("Licence Plate"),
    School: Joi.string().required().label("School"),
    seats: Joi.number().required().label("seats"),
  };

  async componentDidMount() {
    try {
      const { data } = await getBuses();
      this.setState({ busses: data.registeredVehicles });
    } catch (error) {
      console.error("Error fetching users:", error.message);
    }
  }

  doSubmit = async () => {
    try {
      const { data } = this.state;
      await addNewBus({
        vehicleID: data.vehicleID,
        School: data.School,
        seats: data.seats,
        seatsFilled: 0,
        driver: "",
      });
      // Optionally, you can redirect the user or perform other actions after the bus is successfully added.
      console.log("Bus added successfully!");
    } catch (error) {
      console.error("Error adding bus:", error.message);
    }
  };

  render({ busses } = this.state) {
    return (
      <div className="row">
        <div className="col-4">
          <h1>Add a new bus</h1>
          <form onSubmit={this.handleSubmit}>
            {this.renderInput("vehicleID", "Licence Plate")}
            {this.renderInput("School", "School")}
            {this.renderInput("seats", "seats")} <br />
            {this.renderButton("Save")}
          </form>{" "}
          <br />
          <div class="card">
            <div class="card-body">
              <h5 class="card-title">Assign Busses to new users</h5>
              <p class="card-text">You can assign Busses to new users here</p>
              <a href="/assignBusses" class="btn btn-primary">
                Assign Busses
              </a>
            </div>
          </div>
        </div>

        <div className="col">
          <h1>Busses</h1>
          <table class="table">
            <thead>
              <tr>
                <th scope="col">Licence Plate</th>
                <th scope="col">School</th>
                <th scope="col">seats</th>
                <th scope="col">seats filled</th>
              </tr>
            </thead>
            <tbody>
              {busses.map((bus) => (
                <tr key={bus.vehicleID}>
                  <td>{bus.vehicleID}</td>
                  <td>{bus.School}</td>
                  <td>{bus.seats}</td>
                  <td>{bus.seatsFilled}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    );
  }
}

export default Bus;
