args <- commandArgs(trailingOnly = TRUE)

cbind.fill <- function(...){
    nm <- list(...)
    nm <- lapply(nm, as.matrix)
    n <- max(sapply(nm, nrow))
    do.call(cbind, lapply(nm, function (x)
        rbind(x, matrix(, n-nrow(x), ncol(x)))))
}

option = args[1]
args = args[2:length(args)]
count = 0
reading_files = FALSE
servers = c()
files = c()

for (i in 1:length(args))
{
    if (grepl(",", args[i]))
    {
        reading_files = TRUE
    }

    if (!reading_files){
        count = count + 1
    }

    if (reading_files)
    {
        files[i - count] <- sub(',', '', args[i])
    }else
    {
       servers[i] <- args[i]
    }

}

data_folder = "./data/"
plot_folder = "./plots/"

i = 0
while (i < length(files))
{
    quartz()
    min = read.csv(paste(data_folder, files[2 + i], sep=''))
    columns = colnames(min)
    new_column_names = c()

    for (j in 1:length(columns)) {
        new_column_names[j] <- paste(as.character(columns[j]), 'min',sep='_')
    }

    colnames(min) <- new_column_names

    max = read.csv(paste(data_folder, files[3 + i], sep=''))
    columns = colnames(max)
    new_column_names = c()

    for (j in 1:length(columns)) {
        new_column_names[j] <- paste(as.character(columns[j]), 'max',sep='_')
    }

    colnames(max) <- new_column_names

    default_data_files = read.csv(paste(data_folder, files[4 + i], sep=''))
    default_data_files = default_data_files[,c(servers[1], c(servers[2]))]

    comparison_matrix = cbind.fill(min, default_data_files)
    comparison_matrix = cbind.fill(comparison_matrix, max)

    nfp = files[1 + i]

    if(nfp != "mean_cpu_utilization") {
        invalid_data_indices = comparison_matrix > 36
        comparison_matrix[invalid_data_indices] = NA
    }

    png(file = paste(plot_folder, option, '_comparison_', nfp, '.png', sep=''))
    boxplot(comparison_matrix, use.col=TRUE, las=2)
    title(paste(option, '_', nfp, sep=''))
    columns = colnames(comparison_matrix)

    means = c()
    for (j in 1:length(columns)) {
        server = as.character(columns[j])
        data = comparison_matrix[,j]
        means[j] <- mean(data, na.rm=TRUE)
    }

    points(means, col='red', pch=19)
    abline(h=mean(means), col='red', lwd=3, lty=2)

    i = i + 4
    dev.off()
}





