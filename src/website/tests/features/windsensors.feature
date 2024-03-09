@wind-sensors
Feature: Testing the WindSensors API

    Scenario: Fetch WindSensors data from the API
        Given I clear the database
        And I insert WindSensors data into the database
        When I get all WindSensors interface data
        Then the service success response is 200
        And the response data matches the WindSensors data in the database
