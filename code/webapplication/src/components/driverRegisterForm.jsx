import React from "react";
import Joi from "joi-browser";
import Form from "./common/form";
import { registerNewDriver } from "../services/registerService";
//import { notAssignedDriversVehicles } from "../services/registerService";

class RegisterForm extends Form {
  state = {
    data: {
      firstName: "",
      lastDName: "",
      userDname: "",
      password: "",
      contactDNumber: "",
      emailD: "",
      addressD: "",
      nicD: "",
      licensenumberD: "",
      assignedVehicleIdD: "",
    },
    //busses: [],
    errors: {},
  };

  schema = {
    firstName: Joi.string().required().label("First Name"),
    lastDName: Joi.string().required().label("Last Name"),
    userDname: Joi.string().required().label("Username"),
    password: Joi.string().required().min(6).label("Password"),
    contactDNumber: Joi.string().required().label("Contact Number"),
    emailD: Joi.string().required().label("Email"),
    addressD: Joi.string().required().label("Address"),
    nicD: Joi.string().required().min(10).max(12).label("NIC"),
    licensenumberD: Joi.string().required().label("License Number"),
    assignedVehicleIdD: Joi.string().required().label("Bus"),
  };

  // async componentDidMount() {
  //   try {
  //     const { data } = await notAssignedDriversVehicles();
  //     this.setState({ busses: data.notAssignedDriversVehicles });
  //   } catch (error) {
  //     console.error("Error fetching users:", error.message);
  //   }
  // }

  doSubmit = async () => {
    try {
      const { data } = this.state;
      await registerNewDriver({
        firstName: data.firstName,
        lastDName: data.lastDName,
        userDname: data.userDname,
        password: data.password,
        contactDNumber: data.contactDNumber,
        emailD: data.emailD,
        addressD: data.addressD,
        nicD: data.nicD,
        licensenumberD: data.licensenumberD,
        assignedVehicleIdD: data.assignedVehicleIdD,
      });
      // Optionally, you can redirect the user or perform other actions after the bus is successfully added.
      console.log("Driver added successfully!");
    } catch (error) {
      console.error("Error adding bus:", error.message);
    }
  };

  render() {
    return (
      <div>
        <h1>Register a new driver</h1>
        <form onSubmit={this.handleSubmit}>
          {this.renderInput("firstName", "First Name")}
          {this.renderInput("lastDName", "Last Name")}
          {this.renderInput("userDname", "Username")}
          {this.renderInput("password", "Password", "password")}
          {this.renderInput("contactDNumber", "Contact Number")}
          {this.renderInput("emailD", "Email")}
          {this.renderInput("addressD", "Address")}
          {this.renderInput("nicD", "NIC")}
          {this.renderInput("licensenumberD", "License Number")}
          {this.renderInput("assignedVehicleIdD", "Bus")}
          <br />
          {this.renderButton("Register")}
        </form>
      </div>
    );
  }
}

export default RegisterForm;
