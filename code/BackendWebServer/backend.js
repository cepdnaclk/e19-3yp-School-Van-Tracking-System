const express = require("express");
const bcrypt = require("bcrypt");
const mongoose = require("mongoose");
const cors = require("cors");

const app = express();
const port = 3000;

app.use(cors());
app.use(express.json()); // Enable parsing of JSON in requests

// Connect to MongoDB
mongoose.connect(
  "mongodb+srv://musthak:Mk741300@cluster0.zl8gzee.mongodb.net/SureWay2024"
);
const db = mongoose.connection;

db.on("error", console.error.bind(console, "MongoDB connection error:"));
db.once("open", () => {
  console.log("Connected to MongoDB Atlas");
});

// Define a schema for the children
const childSchema = new mongoose.Schema({
  name: String,
  age: Number,
  school: String,
  pickupAddress: String,
  dropAddress: String,
  vehicleID: String,
  travellingStatus: { type: Number },
});

const userSchema = new mongoose.Schema({
  fullName: { type: String, required: true },
  username: { type: String, required: true },
  contactNumber: { type: String, required: true },
  email: { type: String, required: true },
  ChildAddRequest: { type: Number, default: -1 },
  children: [childSchema], // An array of children objects
});

// Define a schema for the driver collection
const driverSchema = new mongoose.Schema({
  firstName: { type: String, required: true },
  lastDName: { type: String, required: true },
  userDname: { type: String, required: true },
  hashedDPassword: { type: String, required: true },
  contactDNumber: { type: String, required: true },
  emailD: { type: String, required: true },
  addressD: { type: String, required: true },
  nicD: { type: String, required: true },
  licensenumberD: { type: String, required: true },
  assignedVehicleIdD: { type: String, default: null },
});

// Define a schema for the vehicle collection
const vehicleSchema = new mongoose.Schema({
  vehicleID: { type: String, required: true },
  School: { type: String, required: true },
  seats: { type: Number, default: 0 },
  seatsFilled: { type: Number, default: 0 },
  Driver: { type: String, default: null },
  children: { type: Array, default: null },
});

// Create a User model based on the schema
const User = mongoose.model("User", userSchema, "Users");

// Create a driver model based on the schema
const driver = mongoose.model("Driver", driverSchema, "Drivers");

// Create a bus model based on the schema
const bus = mongoose.model("Bus", vehicleSchema, "Vehicles");

//------------------------------------methods for users retrieving (read)-----------------------

app.get("/forRegistration", async (req, res) => {
  try {
    // Retrieve users who are need to register from the 'Sureway' collection
    const usersNeedToRegister = await User.find({ ChildAddRequest: 0 }).select(
      "-hashedPassword"
    );

    // Print the data to the console
    console.log("User needs to register:", usersNeedToRegister);

    res.json({
      success: true,
      message: "Data retrieval successful",
      users: usersNeedToRegister,
    });
  } catch (error) {
    console.error(
      "Error during getting users who need to register:",
      error.message
    );
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

app.get("/registeredUsers", async (req, res) => {
  try {
    // Retrieve all registered-users from the 'Sureway' collection
    const registeredUsers = await User.find({ ChildAddRequest: 1 }).select(
      "-hashedPassword"
    );

    // Print the data to the console
    console.log("Registered users:", registeredUsers);

    res.json({
      success: true,
      message: "Data retrieval successful",
      users: registeredUsers,
    });
  } catch (error) {
    console.error("Error during getting registered users:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for users registering (update)-----------------------

app.put("/registering", async (req, res) => {
  const { name } = req.body;
  // console.log("Received request body:", req.body);

  try {
    const existingUser = await User.findOne({ username: name });
    console.log("Existing User:", existingUser);

    if (existingUser) {
      await User.updateOne(
        { username: name },
        { $set: { ChildAddRequest: 1 } }
      );
      res.json({
        success: true,
        message: "User already exists, updated 'ChildAddRequest' field to 1",
      });
    } else {
      res.status(404).json({ success: false, message: "User not found" });
    }
  } catch (error) {
    console.error("Error during registration:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for driver adding (create)-----------------------

app.post("/driverRegistration", async (req, res) => {
  const {
    firstName,
    lastDName,
    userDname,
    password,
    contactDNumber,
    emailD,
    addressD,
    nicD,
    licensenumberD,
    assignedVehicleIdD,
  } = req.body;
  // console.log("Received request body:", req.body);

  try {
    const hashedDPassword = await bcrypt.hash(password, 10);
    console.log("Hashed password:", hashedDPassword);

    // created a new driver Document..
    const newDriver = new driver({
      firstName,
      lastDName,
      userDname,
      hashedDPassword,
      contactDNumber,
      emailD,
      addressD,
      nicD,
      licensenumberD,
      assignedVehicleIdD,
    });

    // Save the User document to the database
    await newDriver.save();

    // You can print the data to the console, including the hashed password
    console.log("Received driver data:", {
      firstName,
      lastDName,
      userDname,
      hashedDPassword,
      contactDNumber,
      emailD,
      addressD,
      nicD,
      licensenumberD,
      assignedVehicleIdD,
    });

    // Send a response to the client
    res.json({ success: true, message: "Driver adding successful" });
  } catch (error) {
    console.error("Error during adding driver:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for vehicle adding (create)-----------------------

app.post("/vehicleRegistration", async (req, res) => {
  const { vehicleID, School, seats, seatsFilled, driver } = req.body;
  console.log("Received request body:", req.body);

  try {
    const newVehicle = new bus({
      vehicleID,
      School,
      seats,
      seatsFilled,
      driver,
    });

    // Save the User document to the database
    await newVehicle.save();

    // You can print the data to the console, including the hashed password
    console.log("Received driver data:", {
      vehicleID,
      School,
      seats,
      seatsFilled,
      driver,
    });

    // Send a response to the client
    res.json({ success: true, message: "Vehicle adding successful" });
  } catch (error) {
    console.error("Error during adding vehicle :", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for vehicle showing (read)-----------------------

app.get("/registeredVehicles", async (req, res) => {
  try {
    // Retrieve all registered-vehicles from the 'Sureway' collection
    const registeredVehicles = await bus.find();

    // Print the data to the console
    console.log("Registered Vehicles:", registeredVehicles);

    res.json({
      success: true,
      message: "Data retrieval successful",
      registeredVehicles: registeredVehicles,
    });
  } catch (error) {
    console.error("Error during getting registered vehicles:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});
//------------------------------------methods for vehicle showing which are not having drivers (read)-----------------------

app.get("/notAssignedDriversVehicles", async (req, res) => {
  try {
    // Retrieve all registered-vehicles from the 'Sureway' collection
    const notAssignedDriversVehicles = await bus.find({ Driver: null });

    // Print the data to the console
    console.log("notAssignedDriversVehicles:", notAssignedDriversVehicles);

    res.json({
      success: true,
      message: "Data retrieval successful",
      notAssignedDriversVehicles: notAssignedDriversVehicles,
    });
  } catch (error) {
    console.error("Error during getting no driver vehicles:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for drivers showing (read)-----------------------

app.get("/gettingDrivers", async (req, res) => {
  try {
    // Retrieve all registered-users from the 'Sureway' collection
    const gettingDrivers = await driver.find();

    // Print the data to the console
    console.log("Getting Drivers:", gettingDrivers);

    res.json({
      success: true,
      message: "Data retrieval successful",
      gettingDrivers: gettingDrivers,
    });
  } catch (error) {
    console.error("Error during getting registered drivers:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for showing children who are not assigned to bus (read)-----------------------

app.get("/childrenDetails", async (req, res) => {
  try {
    const childDetails = await User.find({
      children: {
        $elemMatch: {
          vehicleID: null,
        },
      },
      ChildAddRequest: 1,
    }).select("children");

    // Print the data to the console
    console.log("Child Details:", childDetails);

    res.json({
      success: true,
      message: "Data retrieval successful",
      childDetails: childDetails,
    });
  } catch (error) {
    console.error("Error during getting children:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for assigning bus for children who are not assigned to bus (update)-----------------------

app.put("/assigningVehicle", async (req, res) => {
  const { username, name, vehicleID } = req.body;
  console.log("Received request body:", req.body);
  try {
    const existingChild = await User.findOne({
      children: {
        $elemMatch: {
          name: name,
        },
      },
      username: username,
    });

    console.log("Existing Child:", existingChild);

    if (existingChild) {
      await User.updateOne(
        {
          "children.name": name,
          username: username,
        },
        {
          $set: {
            "children.$.vehicleID": vehicleID,
          },
        }
      );
      await bus.updateOne({ vehicleID }, { $push: { children: name } });

      res.json({
        success: true,
        message: "Child already exists, updated assigned vehicle",
      });
    } else {
      res.status(404).json({ success: false, message: "Child not found" });
    }
  } catch (error) {
    console.error("Error during assigning bus for children:", error.message);
    res.status(500).json({ success: false, message: "Internal Server Error" });
  }
});

//------------------------------------methods for getting location coordinates (read)-----------------------

app.get("/dummyCoordinates", (req, res) => {
  // Assuming dummy coordinates for a location
  const latitude = 37.7749;
  const longitude = -122.4194;

  // Send the coordinates as JSON to the frontend
  res.json({ latitude, longitude });
});

app.listen(port, "0.0.0.0", () => {
  console.log(`Server is running on http://localhost:${port}`);
});
