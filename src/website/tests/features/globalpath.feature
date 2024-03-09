@globalpath
Feature: Testing the GlobalPath API

    Scenario: Fetch GlobalPath data from the API
        Given I clear the database
        And I insert GlobalPath data into the database
        When I get all GlobalPath interface data
        Then the service success response is 200
        And the response data matches the GlobalPath data in the database
