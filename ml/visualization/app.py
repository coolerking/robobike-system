"""Run with streamlit run ml/visualization/app.py."""

import rerun as rr
import streamlit as st


def main():
    st.title("Robobike データキュレーション")
    st.info("TODO: データセットの読み込み、選別、時系列可視化を実装してください。")
    rr.init("robobike-curation", spawn=False)


if __name__ == "__main__":
    main()
