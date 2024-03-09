@batteries
Feature: Testing the Batteries API

    Scenario: Fetch Batteries data from the API
        Given I clear the database
        And I insert Batteries data into the database
        When I get all Batteries interface data
        Then the service success response is 200
        And the response data matches the Batteries data in the database
