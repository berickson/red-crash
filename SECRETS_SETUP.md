# Setting Up API Keys and Secrets

## OpenAI API Key Setup

1. **Create your secrets file:**
   ```bash
   cp secrets.env.template secrets.env
   ```

2. **Edit secrets.env and add your OpenAI API key:**
   ```bash
   nano secrets.env
   ```
   
   Uncomment and replace with your actual key:
   ```bash
   export OPENAI_API_KEY=sk-proj-your-actual-key-here
   ```

3. **Rebuild and restart the Docker container:**
   ```bash
   cd docker
   ./build
   docker stop car
   docker rm car
   ./start
   ```

4. **Verify it's loaded:**
   ```bash
   docker exec car bash -c "echo \$OPENAI_API_KEY"
   ```

## Security Notes

- `secrets.env` is gitignored and will NOT be committed to GitHub
- `secrets.env.template` is committed as a reference template (no actual keys)
- The secrets file is sourced automatically when entering the container
- Never commit actual API keys to version control

## Getting an OpenAI API Key

1. Go to https://platform.openai.com/api-keys
2. Sign in with your OpenAI account
3. Click "Create new secret key"
4. Copy the key (you won't be able to see it again)
5. Add it to your `secrets.env` file
