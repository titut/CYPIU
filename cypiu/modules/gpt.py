"""
Modules for function interaction with OpenAI API
"""

from openai import OpenAI
client = OpenAI()

def parse_command(sentence):
    """
    Takes in a sentence and parses it into action and object using OpenAI api.

    Args:
        sentence: String

    Returns:
        [{action: String}, {object: String}]
    """

    response = client.responses.create(
        model="gpt-4.1-nano-2025-04-14",
        input="I give you sentence (command). Break it down into action and object. Example: 'pick up an apple'. Returns: 'pick up, apple'. Returns always have form: '{action}, {object}'. Make sure object is strictly just the noun. \n Sentence: " + sentence
    )

    command = response.output_text.replace(" ", "").split(",")
    return command