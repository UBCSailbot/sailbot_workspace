@ph-sensors
Feature: Testing the PhSensors API

    Scenario: Fetch PhSensors data from the API
        Given I clear the database
        And I insert PhSensors data into the database
        When I get all PhSensors interface data
        Then the service success response is 200
        And the response data matches the PhSensors data in the database
