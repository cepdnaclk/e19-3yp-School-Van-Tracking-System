import React, { Component } from "react";
import SearchBox from "./common/searchBox";
import { getDrivers } from "../services/driverRecordsservices";
import userbanner from "../images/userbanner.jpg";
import { Route } from "react-router-dom";

class DriverRecord extends Component {
  state = {
    drivers: [],
    selectedDriver: "",
    searchQuery: "",
  };

  async componentDidMount() {
    try {
      const { data } = await getDrivers();
      this.setState({ drivers: data.gettingDrivers });
      this.setState({ selectedDriver: data.gettingDrivers[0] });
    } catch (error) {
      console.error("Error fetching users:", error.message);
    }
  }

  handleUserSelect = (user) => {
    this.setState({ selectedDriver: user });
  };

  handleSearch = (query) => {
    this.setState({ searchQuery: query });
  };

  filterUsers = () => {
    const { drivers, searchQuery } = this.state;
    if (!searchQuery) return drivers;

    const filtereddrivers = drivers.filter((user) =>
      user?.firstName?.includes(searchQuery)
    );

    const remainingdrivers = drivers.filter(
      (user) => !filtereddrivers?.includes(user)
    );

    return [...filtereddrivers, ...remainingdrivers];
  };

  render() {
    const { selectedDriver, searchQuery } = this.state;
    const filteredDrivers = this.filterUsers();

    return (
      <>
        <div className="row">
          <div className="col-3">
            <div
              className="card"
              style={{
                marginTop: "20px",
                marginLeft: "20px",
                width: "400px",
                padding: "20px",
                borderRadius: "20px",
                height: "610px",
                border: "none",
                boxShadow: "0px 0px 10px rgba(0, 0, 0, 0.1)",
                transition: "transform 0.2s, background-color 0.2s",
              }}
            >
              <ul className="list-group" style={{ marginTop: 16 }}>
                {filteredDrivers &&
                  filteredDrivers.map((user) => (
                    <li
                      key={user._id}
                      className={
                        user === selectedDriver
                          ? "list-group-item active"
                          : "list-group-item"
                      }
                      onClick={() => this.handleUserSelect(user)}
                    >
                      {user.firstName}
                    </li>
                  ))}
              </ul>
            </div>
          </div>
          <div
            className="card"
            style={{
              margin: "20px",
              marginLeft: "120px",

              width: "870px",
              padding: "20px",
              borderRadius: "20px",
              height: "610px",
              border: "none",
              boxShadow: "0px 0px 10px rgba(0, 0, 0, 0.1)",
              transition: "transform 0.2s, background-color 0.2s",
            }}
          >
            <div className="col">
              <SearchBox value={searchQuery} onChange={this.handleSearch} />
              {selectedDriver ? (
                <div
                  className="card mb-3"
                  style={{
                    margin: "30px",
                    padding: "15px",
                    borderRadius: "10px",
                    height: "300px",
                    border: "none",
                    width: "600px",
                    boxShadow: "0px 0px 10px rgba(0, 0, 0, 0.1)",
                    backgroundColor: "#ffffff",
                    transition: "transform 0.2s, background-color 0.2s",
                    overflowY: "auto",
                  }}
                >
                  <div className="row g-0">
                    <div className="col-md-4">
                      <img
                        src={userbanner}
                        className="img-fluid rounded-start"
                        alt="..."
                        style={{
                          maxWidth: "100%",
                          maxHeight: "100%",
                          minWidth: "100%",
                          minHeight: "100%",
                        }}
                      />
                    </div>
                    <div className="col-md-8">
                      <div className="card-body">
                        <h5 className="card-title">
                          {selectedDriver.firstName || "No drivers"}
                        </h5>
                        <p className="card-text">
                          UserName: {selectedDriver.userName} <br />
                          Contact Number: {selectedDriver.contactNumber} <br />
                          Email: {selectedDriver.email} <br />
                          Address: {selectedDriver.address} <br />
                          NIC: {selectedDriver.NIC} <br />
                          Licence Number: {selectedDriver.licenseNumber} <br />
                          Bus: {selectedDriver.assignedVehicle} <br />
                          <Route
                            to="/seeSnaps"
                            render={(props) => (
                              <a
                                href="/seeSnaps"
                                className="btn btn-warning"
                                onClick={() =>
                                  props.history.push("/seeSnaps", {
                                    selectedDriver,
                                  })
                                }
                              >
                                See Snaps
                              </a>
                            )}
                          />
                        </p>
                      </div>
                    </div>
                  </div>
                </div>
              ) : (
                <p>No drivers available</p>
              )}
            </div>
          </div>
        </div>
      </>
    );
  }
}

export default DriverRecord;
