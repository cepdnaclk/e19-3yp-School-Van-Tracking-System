import React from "react";

const RegisterUsers = () => {
  return (
    <div class="row row-cols-1 row-cols-md-3 g-4">
      <div class="card">
        <div class="card-body">
          <h5 class="card-title">Driver Registration</h5>
          <p class="card-text">You can register drivers here</p>
          <a href="/driverRegisterForm" class="btn btn-primary">
            Register
          </a>
        </div>
      </div>
      {/* <div class="card">
        <div class="card-body">
          <h5 class="card-title">Ambulance Service registration</h5>
          <p class="card-text">You can register mbulance ervices here</p>
          <a href="/ambulanceRegisterForm" class="btn btn-primary">
            Register
          </a>
        </div>
      </div> */}
    </div>
  );
};

export default RegisterUsers;
