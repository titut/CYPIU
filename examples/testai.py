from openai import OpenAI
client = OpenAI()

sentence = "Can you grab the object from my room?"

response = client.responses.create(
    model="gpt-4.1-nano-2025-04-14",
    input="I give you sentence (command). Break it down into action and object. Example: 'pick up an apple'. Returns: 'pick up, apple'. Returns always have form: '{action}, {object}'. Make sure do not include articles. \n Sentence: " + sentence
)

command = response.output_text.replace(" ", "").split(",")
print(command)