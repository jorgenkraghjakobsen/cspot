#include "AudioChunkManager.h"

AudioChunkManager::AudioChunkManager()
{
    this->chunks = std::vector<std::shared_ptr<AudioChunk>>();
    startTask();
    //this->audioChunkDataQueue = new Queue<std::pair<std::vector<uint8_t>, bool>>;
}

std::shared_ptr<AudioChunk> AudioChunkManager::registerNewChunk(uint16_t seqId, std::vector<uint8_t> &audioKey, uint32_t startPos, uint32_t endPos)
{
    auto chunk = std::make_shared<AudioChunk>(seqId, audioKey, startPos * 4, endPos * 4);
    this->chunks.push_back(chunk);
    printf("Chunk requested %d\n", seqId);

    return chunk;
}
void AudioChunkManager::handleChunkData(std::vector<uint8_t>& data, bool failed)
{
    auto audioPair = std::pair(data, failed);
    audioChunkDataQueue.push(audioPair);
}

void AudioChunkManager::runTask() {
    while(true) {
        std::pair<std::vector<uint8_t>, bool> audioPair;
        if (this->audioChunkDataQueue.wpop(audioPair)) {
            auto data = audioPair.first;
            auto failed = audioPair.second;
            uint16_t seqId = ntohs(extract<uint16_t>(data, 0));

            // Erase all chunks that are not referenced elsewhere anymore
            chunks.erase(std::remove_if(
                     chunks.begin(), chunks.end(),
                     [](const std::shared_ptr<AudioChunk> &chunk) {
                         return chunk.use_count() == 1;
                     }),
                 chunks.end());

            for (auto const &chunk : this->chunks)
            {
                // Found the right chunk
                if (chunk->seqId == seqId)
                {
                    if (failed)
                    {
                        chunk->isHeaderFileSizeLoadedSemaphore->give();
                        chunk->isLoadedSemaphore->give();
                        break;
                    }

                    switch (data.size())
                    {
                    case DATA_SIZE_HEADER:
                    {
                        printf("ID: %d: header decrypt!\n", seqId);
                        auto headerSize = ntohs(extract<uint16_t>(data, 2));
                        // Got file size!
                        chunk->headerFileSize = ntohl(extract<uint32_t>(data, 5)) * 4;
                        chunk->isHeaderFileSizeLoadedSemaphore->give();
                        break;
                    }
                    case DATA_SIZE_FOOTER:
                        if (chunk->endPosition > chunk->headerFileSize)
                        {
                            chunk->endPosition = chunk->headerFileSize;
                        }
                        printf("ID: %d: Starting decrypt!\n", seqId);
                chunk->decrypt();
                chunk->isLoadedSemaphore->give();
                break;

            default:
                // printf("ID: %d: Got data chunk!\n", seqId);
                // 2 first bytes are size so we skip it
                auto actualData = std::vector<uint8_t>(data.begin() + 2, data.end());
                chunk->appendData(actualData);
                break;
            }
        }
    }
    }
}
}

