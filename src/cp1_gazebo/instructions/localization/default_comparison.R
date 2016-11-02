# WARNING: This file might not work if the name of the documents do not match the ones we provide here

data_folder = "./data/"
default_data_files = c('mean_amcl_cpu_utilization_default_data', 'mean_cpu_utilization_default_data',
                    'mean_localization_error_default_data', 'time_default_data')

for (i in 1:length(default_data_files)) {
    quartz()
    matrix = read.csv(paste(data_folder, default_data_files[i], '.csv', sep=''))

    invalid_data_indices = matrix > 36
    matrix[invalid_data_indices] = NA

    columns = colnames(matrix)

    boxplot(matrix, use.col=TRUE, las=2)
    title(default_data_files[i])
    means = c()
    for (j in 1:length(columns)) {
        server = as.character(columns[j])
        data = matrix[,server]
        means[j] <- mean(data, na.rm=TRUE)
    }

    points(means, col='red')
    abline(h=mean(means), col='red', lwd=3, lty=2)
}