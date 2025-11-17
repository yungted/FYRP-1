import moondream as md
from PIL import Image

# Replace with your actual API key
API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJrZXlfaWQiOiIxZWI5Njk0MS02MDVlLTQ3ZTYtYjI5NS01NjdjMGRjYWZhYTkiLCJvcmdfaWQiOiJlZzJ2bXo4a1FWY0xlOTExQVl5bkM0NU44YkhEYTMwTyIsImlhdCI6MTc2MjkzMzIzOSwidmVyIjoxfQ.onaMxtqh-X14ggd8XUDfiALxzYuHuUyI3GMbWHBYCqw"

# Connect to the cloud
model = md.vl(endpoint="https://api.moondream.ai/v1", api_key=API_KEY)

# Load an image
image_path = r"C:\Users\User\Downloads\images.jpeg"
image = Image.open(image_path)

# Query safely
response = model.query(image, "What's in this image?")
answer = response.get("answer", "No answer returned")
print("Answer:", answer)
