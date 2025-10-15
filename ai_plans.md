# AI Integration Plans

## Completed

- [x] Added OpenAI SDK to command_interpreter.py as fallback for unknown commands
- [x] Integrated OpenAI API with basic text responses (gpt-4o-mini)
- [x] Set up secrets.env for API key management (gitignored)
- [x] Added OpenAI to Dockerfile pip install
- [x] Basic conversational AI working through speech_ros

## Next Steps for Vision

### Goal: Add Camera Vision to AI Responses
Enable robot to see trick-or-treaters and respond with visual context.

### Implementation Plan

1. **Add camera subscription to command_interpreter.py**
   - Subscribe to `/oak/rgb/image_raw` (sensor_msgs/Image)
   - Store latest image in class variable
   - Add cv_bridge dependency for image conversion

2. **Modify ask_openai() to accept images**
   - Encode image to base64 for OpenAI API
   - Use vision-capable model (gpt-4o or gpt-4o-mini with vision)
   - Include image in API request when available
   - Add image compression to reduce API costs

3. **Add vision toggle**
   - Parameter to enable/disable vision mode
   - Only send images when explicitly needed (save tokens/costs)

4. **Test and refine**
   - Test with camera feed
   - Adjust system prompt for trick-or-treat scenario
   - Monitor token usage and costs

## Open Questions

- Should conversation context reset after period of inactivity?
- What system prompt to use for trick-or-treat persona?
- Should we log conversations for debugging/improvement?
- Image resolution/quality tradeoff for API costs?
- Should vision be always-on or only when requested?
