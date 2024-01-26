const express = require("express");
const cors = require("cors");

const app = express();
const port = 3000;
app.use(cors());
app.use(express.urlencoded({ extended: true }));

app.use(express.json()); // Enable parsing of JSON in requests

const connectToDatabase = require("./mongoDB/connection");
connectToDatabase();

const verifyAdmin = require("./adminConfigurations/verifyingAdmin");

app.post("/verifyingAdmin", async (req, res, next) => {
  const { body } = req;
  const { username, password, verificationCode } = body;

  const verificationResult = await verifyAdmin(
    username,
    password,
    verificationCode
  );

  if (verificationResult.success) {
    res.send(verificationResult.success);
  } else {
    res.status(400).send(verificationResult.error);
  }
});

const adminLogin = require("./adminConfigurations/adminLogin");
app.use("/Admin", adminLogin);

const adminSignup = require("./adminConfigurations/adminSignup");
app.use("/Admin", adminSignup);

const getRegisteredUsers = require("./admin/gettingRegisteredUsers");
app.use("/Admin", getRegisteredUsers);

const getDrivers = require("./admin/gettingDrivers");
app.use("/Admin", getDrivers);

const addingDrivers = require("./admin/driverRegistration");
app.use("/Admin", addingDrivers);

const addingVehicles = require("./admin/vehicleRegistration");
app.use("/Admin", addingVehicles);

const gettingVehicles = require("./admin/gettingRegisteredVehicles");
app.use("/Admin", gettingVehicles);

const gettingbusNotAssignedChildren = require("./admin/gettingbusNotAssignedChildren");
app.use("/Admin", gettingbusNotAssignedChildren);

const gettingThingName = require("./admin/gettingThingName");
app.use("/Admin", gettingThingName);

app.listen(port, "0.0.0.0", () => {
  console.log(`Server is running on http://localhost:${port}`);
});