import streamlit as st
from PIL import Image
import matplotlib.pyplot as plt

st.set_page_config(layout="wide")
st.title("🖼️ Image Keypoint Visualizer")

# ---------- Session State ----------
if "render" not in st.session_state:
    st.session_state.render = False

if "points" not in st.session_state:
    st.session_state.points = []

if "image" not in st.session_state:
    st.session_state.image = None


# ---------- Layout ----------
left_col, right_col = st.columns(2)

# ---------- Left: Inputs ----------
with left_col:
    st.header("Inputs")

    uploaded_image = st.file_uploader(
        "Upload an image",
        type=["png", "jpg", "jpeg"]
    )

    coord_text = st.text_area(
        "Enter keypoints (x,y per line)",
        height=200,
        placeholder="100,200\n300,400\n500,600"
    )

    refresh = st.button("🔄 Refresh / Redraw")

    if refresh:
        if uploaded_image is not None:
            st.session_state.image = Image.open(uploaded_image).convert("RGB")

            points = []
            for line in coord_text.splitlines():
                try:
                    x, y = map(float, line.split(","))
                    points.append((x, y))
                except:
                    pass

            st.session_state.points = points
            st.session_state.render = True
        else:
            st.warning("Please upload an image before refreshing")

# ---------- Right: Output ----------
with right_col:
    st.header("Rendered Output")

    if st.session_state.render and st.session_state.image is not None:
        image = st.session_state.image
        width, height = image.size

        fig, ax = plt.subplots()
        ax.imshow(image)
        ax.axis("off")

        if st.session_state.points:
            xs, ys = zip(*st.session_state.points)
            ax.scatter(xs, ys, c="red", s=50)
            ax.plot(xs, ys, c="red", linewidth=2)

        ax.set_title(f"Image Resolution: {width} x {height}")
        st.pyplot(fig)
    else:
        st.info("Click Refresh to render keypoints")
